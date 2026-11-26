"""Robot-local durable photo handoff. Knows capture IDs, never mission context."""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import threading
import time
from urllib.parse import quote, urlparse
from urllib.request import build_opener, HTTPRedirectHandler, ProxyHandler, Request
from uuid import uuid4


class NoRedirects(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class CaptureDelivery:
    def __init__(self, directory, robot_name, *, token='', logger=None, start_worker=True):
        Path(directory).mkdir(parents=True, exist_ok=True)
        self.robot_name = robot_name
        self.token = token
        self.logger = logger
        self._lock = threading.RLock()
        self._db = sqlite3.connect(Path(directory) / 'outbox.sqlite3', check_same_thread=False)
        self._db.execute('PRAGMA synchronous=FULL')
        self._db.execute('CREATE TABLE IF NOT EXISTS captures (capture_id TEXT PRIMARY KEY, metadata TEXT NOT NULL, jpeg BLOB, sha256 TEXT NOT NULL, delivered INTEGER NOT NULL DEFAULT 0)')
        self._db.commit()
        self._descriptor = None
        self._received = 0.0
        self._stop = threading.Event()
        self._worker = None
        self._last_error = None
        self._opener = build_opener(ProxyHandler({}), NoRedirects())
        if start_worker:
            self._worker = threading.Thread(target=self._work, name='polaris-capture-delivery', daemon=True)
            self._worker.start()

    def discover(self, text):
        value = json.loads(text)
        if not isinstance(value, dict):
            raise ValueError('collector descriptor must be an object')
        if value.get('schema_version') != 1 or value.get('robot_name') != self.robot_name:
            raise ValueError('collector descriptor does not match this robot')
        if value.get('ready') is not True:
            with self._lock:
                self._descriptor = None
            return
        parsed = urlparse(value.get('base_url', ''))
        if parsed.scheme not in ('http', 'https') or not parsed.hostname or not parsed.port or parsed.username or parsed.password or parsed.query or parsed.fragment or parsed.path not in ('', '/') or parsed.hostname in ('0.0.0.0', '::'):
            raise ValueError('invalid collector address')
        lease = float(value.get('lease_seconds', 0))
        if not math.isfinite(lease) or lease <= 0:
            raise ValueError('invalid collector lease')
        if value.get('auth', 'none') not in ('none', 'bearer'):
            raise ValueError('unsupported collector authentication')
        endpoints = value.get('endpoints')
        if not isinstance(endpoints, dict):
            raise ValueError('collector endpoints must be an object')
        path = endpoints.get('capture_picture_template')
        if path != '/captures/{capture_id}/picture':
            raise ValueError('collector does not support direct capture delivery')
        with self._lock:
            self._descriptor = value
            self._received = time.monotonic()

    def has_capture(self, capture_id):
        with self._lock:
            return self._db.execute('SELECT 1 FROM captures WHERE capture_id=?', (capture_id,)).fetchone() is not None

    def enqueue(self, capture_id, jpeg, metadata):
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', capture_id):
            raise ValueError('invalid capture ID')
        canonical = json.dumps(metadata, sort_keys=True, separators=(',', ':'), allow_nan=False)
        digest = hashlib.sha256(jpeg).hexdigest()
        with self._lock:
            row = self._db.execute('SELECT metadata,sha256 FROM captures WHERE capture_id=?', (capture_id,)).fetchone()
            if row and row != (canonical, digest):
                raise ValueError('capture ID already contains different bytes or metadata')
            self._db.execute('INSERT OR IGNORE INTO captures (capture_id,metadata,jpeg,sha256) VALUES (?,?,?,?)', (capture_id, canonical, jpeg, digest))
            self._db.commit()

    def deliver_pending(self):
        with self._lock:
            descriptor = self._descriptor
            if not descriptor or time.monotonic() - self._received >= float(descriptor['lease_seconds']):
                return
            pending = self._db.execute('SELECT capture_id,metadata,jpeg,sha256 FROM captures WHERE delivered=0 LIMIT 10').fetchall()
        if descriptor.get('auth') == 'bearer' and not self.token:
            raise RuntimeError('collector credentials are not configured')
        errors = []
        for row in pending:
            try:
                self._deliver_one(descriptor, *row)
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise errors[0]

    def _deliver_one(self, descriptor, capture_id, metadata, jpeg, digest):
        boundary = uuid4().hex
        body = (f'--{boundary}\r\nContent-Disposition: form-data; name="metadata"\r\n\r\n{metadata}\r\n'
                f'--{boundary}\r\nContent-Disposition: form-data; name="artifact"; filename="capture.jpg"\r\nContent-Type: image/jpeg\r\n\r\n').encode() + jpeg + f'\r\n--{boundary}--\r\n'.encode()
        headers = {'Content-Type': f'multipart/form-data; boundary={boundary}'}
        if self.token:
            headers['Authorization'] = f'Bearer {self.token}'
        url = descriptor['base_url'].rstrip('/') + '/captures/' + quote(capture_id, safe='') + '/picture'
        with self._opener.open(Request(url, data=body, headers=headers), timeout=2) as response:
            receipt = json.loads(response.read(65536))
        if receipt.get('capture_id') != capture_id or receipt.get('durable') is not True or receipt.get('sha256') != digest:
            raise RuntimeError('collector did not acknowledge this capture durably')
        with self._lock:
            self._db.execute('UPDATE captures SET delivered=1,jpeg=NULL WHERE capture_id=?', (capture_id,))
            self._db.commit()
        if self.logger:
            self.logger.info(f'Capture {capture_id} handed directly to the collector ({receipt["state"]})')

    def _work(self):
        while not self._stop.wait(1):
            try:
                self.deliver_pending()
                self._last_error = None
            except Exception as exc:
                if self.logger and str(exc) != self._last_error:
                    self.logger.warning(f'Capture retained locally; collector handoff will retry: {exc}')
                self._last_error = str(exc)

    def close(self):
        self._stop.set()
        if self._worker:
            self._worker.join()
        self._db.close()
