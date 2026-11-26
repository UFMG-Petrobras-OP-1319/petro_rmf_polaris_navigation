# Scout Mini hardware motion tuning — November 12, 2026

Goal: reduce steering overshoot/zig-zag while retaining the verified inspection camera lifecycle and normal robot startup.

| Parameter | Physical Scout trial | Simulation default |
|---|---:|---:|
| controller.speed_ref | 0.20 m/s | 0.30 m/s |
| controller.convergence_gain | 4.0 | 8.0 |
| follower_control.const_vel | 0.20 m/s | 0.30 m/s |
| follower_control.const_omega | 1.0 rad/s | 2.0 rad/s |

The values live in `hardware` blocks in `config/scout_params.yaml`. `navigation.launch.py` resolves the controller block and `follower_control.launch.py` resolves the follower block when `use_sim_time=false`. Both strip the hardware mapping before passing ROS parameters. Explicit speed_ref/convergence_gain launch overrides still take precedence. Pioneer and simulation values remain unchanged.

Control-point distance remains 0.15 m, kp_pos/kp_orient remain 1, and the 0.05 s update period is retained. The lower angular saturation applies to both path tracking and final heading alignment. No acceleration limiter, obstacle logic, navigation FSM, camera-home coordinate, camera settling value or adapter behavior was changed.

The operator tested these values on the physical Scout Mini on November 12, 2026 and reported that motion is much better. This is qualitative hardware validation of improved smoothness, not a quantified trajectory comparison or formal safety certification. The tuning aims to reduce aggressive corrections and steering overshoot through lower speed, lower convergence gain and lower angular saturation. The previous bag lacks actual odometry/TF/cmd_vel data; a future recorded comparison can quantify path error and steering reversals. Slower motion may change RMF ETA estimates.

Automated validation before the operator test: 127 inspection tests and 129 joystick-variant tests passed; both overlays built and installed launch parameters were checked. No tuning value changed after that test. No additional hardware commands were issued while recording this operator result.

Run with the existing Scout Compose/container/run.sh workflow; no new launcher is required. Compare the same familiar route at these settings. No base motion was executed while preparing this trial.

Rollback: restore the previous overlay's installed Polaris package from the saved pre-trial archive, or check out the prior Polaris pin and rebuild polaris_control in the same overlay. Do not switch to an unrelated legacy install merely to roll back tuning, because that could drop the calibrated camera-home fixes.
