# Scout robot photo, re-aim and return lifecycle

This implementation is based on the proven inspection Polaris revision
`5ff62da63c991f3e30a21a48e2b71c8cc760d21e`, which contains `CaptureDelivery` and
its durable outbox. It changes robot-side `picture_taker` behavior only; no RMF
adapter or manager code is required.

On Scout hardware, the PTZ driver's raw Hikvision pose `160°/15°/1x` is mapped
to logical `(pan=0 rad, tilt=0 rad, zoom=1x)`. The picture lifecycle waits for
a still base, fresh camera feedback, a fresh expected joint target, and the camera
service. If the manager repeats its cached aim after the prior photo returned
home, Polaris sends the expected pan/tilt commands once, then waits for fresh
settling feedback. It does not keep fighting a separate PTZ owner.

The JPEG and shutter-time pose/joints are persisted to CaptureDelivery before
return begins. The robot then sends one atomic `(pan, tilt, direct_zoom)` home
target, waits for fresh stamped feedback on all three values, and only then
publishes the existing successful photo result. A return failure leaves the
capture in the outbox and reports failure. Capture failures also trigger a
bounded home attempt when hardware return is enabled. A new photo request is
rejected while a lifecycle is active.

Request subscriptions use reliable volatile QoS so reconnect cannot replay a
historical movement request. Collector service discovery keeps its own reliable
transient-local QoS so late subscribers can discover the descriptor. The wire
request and result schemas are unchanged. The 8-second hardware readiness
admission, 5-second camera service timeout, 10-second return timeout, and 0.5
second response margin fit inside the manager's usual 30-second request budget.
An explicitly shorter budget fails before a new aim if there is not enough room.

Simulation and other robot configurations keep their own camera zero and have
return disabled unless separately calibrated. Scout's mount calibration is in
`scout_params.yaml`'s hardware override only. The return pose is logical zero;
raw offsets must not be duplicated in Polaris.

Offline validation in the camera-home worktree: 113 Polaris tests passed for the
inspection variant and 115 passed for the joystick variant. These include
picture lifecycle, durable-delivery integration, launch selection, logical
home, one-shot re-aim, and the STOPPED/teleop ownership tests. No ROS node or
physical camera was operated. Runtime QoS interoperability under the deployed
`rmw_zenoh_cpp`, camera settling, and the full RMF mission remain hardware tests
and require explicit operator approval. The existing manager still releases
HOLD on a failed result, and the wire protocol has no request expiry token or
cancel command if the robot process is killed.

Keep one PTZ command owner during inspection. External joystick/operator camera
control is not arbitrated by this photo lifecycle; it may cause a bounded return
failure rather than an endless command fight. Do not start a robot runtime or
camera from package installation, overlay selection, node startup, or reconnect.
