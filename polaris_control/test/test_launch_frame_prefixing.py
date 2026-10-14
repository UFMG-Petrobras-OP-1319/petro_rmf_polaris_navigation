"""Regression tests for namespaced TF frame launch arguments."""

import importlib.util
from pathlib import Path

import pytest


LAUNCH_DIRECTORY = Path(__file__).parents[1] / 'launch'


def _load_launch_module(filename):
    module_name = filename.removesuffix('.launch.py').replace('.', '_')
    spec = importlib.util.spec_from_file_location(
        module_name, LAUNCH_DIRECTORY / filename
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(
    params=['navigation.launch.py', 'follower_control.launch.py']
)
def launch_module(request):
    return _load_launch_module(request.param)


@pytest.mark.parametrize(
    ('namespace', 'frame', 'prefix_enabled', 'expected'),
    [
        ('scout_1', 'body', True, 'scout_1/body'),
        ('scout_1', 'scout_1/body', True, 'scout_1/body'),
        ('scout_1', 'map', True, 'scout_1/map'),
        ('scout_1', 'scout_1/map', True, 'scout_1/map'),
        ('scout_1', 'scout_10/body', True, 'scout_1/scout_10/body'),
        ('fleet/scout_1', 'body', True, 'fleet/scout_1/body'),
        (
            'fleet/scout_1',
            'fleet/scout_1/body',
            True,
            'fleet/scout_1/body',
        ),
        ('scout_1', '{robot_name}', True, 'scout_1'),
        ('scout_1', 'sim_world', True, 'sim_world'),
        ('scout_1', 'body', False, 'body'),
        ('', 'body', True, 'body'),
        ('scout_1', '', True, ''),
    ],
)
def test_resolve_tf_frame_is_idempotent(
    launch_module, namespace, frame, prefix_enabled, expected
):
    assert launch_module._resolve_tf_frame(
        namespace, frame, prefix_enabled
    ) == expected
