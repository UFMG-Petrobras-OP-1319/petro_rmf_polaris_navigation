# Inspection camera return to forward

This change is based on the robot's original inspection Polaris revision 5ff62da.
It preserves CaptureDelivery and its durable outbox. It does not start any node
or operate hardware. Return is disabled in scout_params.yaml until calibration.

Enable only after an operator has approved camera use and verified the actual
forward view on the mount. Configure the inspection camera under picture_taker:

```yaml
inspection:
  return_to_forward: true
  forward_pose_calibrated: true
  forward_pan_rad: <verified joint-space forward angle>
  forward_tilt_rad: <verified joint-space forward angle>
  pan_cmd_topic: pan_camera_cmd
  tilt_cmd_topic: tilt_camera_cmd
  return_timeout_sec: 10.0
  return_tolerance_rad: 0.02
  return_command_topic: hk_camera_ptz/joint_absolute
  return_position_topic: hk_camera_ptz/joint_position
  forward_zoom: 1.0
```

Angles are joint coordinates after the PTZ driver's sign/offset mapping, not raw
ISAPI angles. A zero offset in the driver is not evidence of chassis-forward
calibration. Keep motor topics and feedback under the same robot namespace.
The original PTZ driver attempts to preserve zoom through its legacy absolute
endpoint. The first approved zero check changed zoom despite that copy. The
extended API subsequently preserved direct zoom 16 during a pan adjustment and
accepted the operator's requested unzoomed view at direct zoom 1. Use the PTZ
driver companion with joint_absolute_api set to extended for this Scout.
The configured atomic command and stamped feedback topics verify zoom as well
as both axes; split-axis return topics remain available for other deployments.
Keep return disabled until the operator selects the final home pose and the
patched driver is installed and tested in an approved camera-only session.

The robot waits for fresh feedback and its command subscriber(s) before sending
one absolute target. It does not repeatedly fight a head that is not
moving. It confirms both axes continuously over settle_sec using distinct fresh
samples. Missing/stale feedback, missing subscribers, or an unsuccessful return
produce a failed result rather than successful completion. A JPEG already taken
stays on disk on failure and is already in the durable collector outbox.

The return runs after capture failure too. The original capture failure remains
a failure even if returning succeeds. Another request is rejected while this
lifecycle owns the camera. No startup movement or robot-base command is added.
Process termination and externally canceled missions have no explicit cancel
message in the existing photo protocol; this change cannot guarantee cleanup
after the node is killed or arbitrate a separate controller commanding the head.
Use one PTZ owner during inspection. The manager's existing failure policy still
releases the base when it receives a failed result.

The fleet-side companion patch consumes resting_joints so its next photo request
does not expect the previous inspection angle. Shutter metadata stays in joints.
Deploy both changes together. The request's timeout_sec bounds readiness,
capture and return; the robot reserves service/return time and a 0.5 s response
margin. With the usual 30 s budget this leaves 14.5 s for readiness, 5 s for the
photo service, and 10 s for the return. A slow physical head may require a larger
fleet picture_timeout_sec and matching return_timeout_sec after measurement.

Do not replace the running installation or switch branches during a mission.
Build to a separate overlay while stopped and source it in a fresh shell only
after calibration and an approved camera-only test. Preserve the working install,
camera.env, original backup branches and stashes. Test the camera in isolation
before any end-to-end mission; running full robot.launch.py also starts base
drivers and is not a camera-only test.
