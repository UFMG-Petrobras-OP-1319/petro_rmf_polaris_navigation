# Changelog

---

## 2026-10-18 - Follower Control: PAN_TILT State (#593)

### Changed

**`polaris_control/src/follower_control.py`**

- `/stop_control` now accepts `PAN_TILT`: entering it clears the planner and holds the base still (zero `cmd_vel`, `vec_to_follow` ignored). Only `CONTROL_POSITION` leaves `PAN_TILT`; `ALIGN_YAW` is ignored there.
- Requires `petro_rmf_utils` with `FollowerControlState.PAN_TILT` (>= `ad3af42`).

---

## 2026-05-19 - Dual-Mode Follower Control: Orientation-Only P Controller

### Overview

`follower_control.py` now supports two mutually exclusive runtime behaviours selectable via a ROS 2 topic flag. The original feedback-linearisation vector-follow mode is preserved exactly. A new proportional orientation-only mode was added to allow the robot to rotate in place to face a commanded target point without translating.

### Added

**`polaris_control/src/follower_control.py`** — new file (node `vector_follower_node`)

- Added two new subscriptions controlled by parameters:
  - `orient_flag_topic` (default `/orient_mode`, type `std_msgs/Bool`): when `True`, switches the node into orientation mode; `False` (default) keeps the existing vec-to-follow behaviour.
  - `orient_point_topic` (default `/orient_target`, type `geometry_msgs/Point`): global-frame (x, y) point the robot should face while in orientation mode.
- Added new parameter `kp_orient` (default `2.0`): proportional gain for the yaw error in orientation mode.
- Orientation controller computes `theta_des = atan2(dy, dx)` from the robot's current position to the target point, normalises the angular error through `atan2(sin(e), cos(e))` to handle wrap-around, and outputs `w = clip(kp_orient * e_theta, ±const_omega)` with `linear.x = 0`.
- Robot position (`robot_x`, `robot_y`) is now tracked alongside `theta` in all three existing pose-source paths: `Odometry_callback`, `amcl_pose_callback`, and `_update_theta_from_tf`, so orientation mode works regardless of the configured `pose_topic_type`.
- When orientation mode is active but the target or pose is not yet available, the node publishes a zero `Twist` and logs a throttled warning.

**`polaris_control/launch/follower_control.launch.py`** — new file

- Standalone launch file that starts the `follower_control.py` node with parameters loaded from `pioneer_params.yaml`.

### Changed

**`polaris_control/CMakeLists.txt`**

- Added install rule for `src/follower_control.py` so the node is available as a ROS 2 executable after build.

**`polaris_control/config/pioneer_params.yaml`**

- Added `follower_control` namespace with the full parameter set for the new node (`distancia_ponto_controle`, `const_vel`, `const_omega`, `control_period`, topic names, pose source configuration).
- Added `planner` namespace (`pose_topic_type: TFMessage`, `tf_reference_frame: map`, `tf_robot_pose: body`) so `path_from_points` reads its frame configuration from this file.
- Updated `controller` and `feedback_linearization` parameter blocks to match Pioneer-specific tuning (`speed_ref: 0.15`, `convergence_gain: 2.0`, explicit `goal_tolerance`, `goal_hysteresis`, `slowdown_radius`, `min_tracking_speed`).

**`polaris_control/launch/feedback_linearization.launch.py`** and **`polaris_control/launch/navigation.launch.py`**

- Changed the default parameter file from `scout_params.yaml` to `pioneer_params.yaml` so Pioneer robot launches pick up the correct configuration without extra arguments.

---

## 2026-05-06 - Obstacle Avoidance Tuning for Scout Mini

### Changed

- `polaris_control/config/closest_obstacle_detector_params.yaml`: tuned obstacle detector parameters for Scout Mini platform.
- `polaris_control/config/scout_params.yaml`: updated controller gains and obstacle avoidance thresholds.
- `polaris_control/launch/safe_nav_stack.launch.py`: revised launch to wire obstacle detector and navigation stack correctly for Scout Mini.

---

## 2026-05-05 - Package Restructure and polaris_planning Introduction

### Overview

The repository was restructured from a flat package layout into two well-defined sub-packages: `polaris_control` and `polaris_planning`. `polaris_planning` is an entirely new addition providing reference path generation nodes. All existing control code was migrated into `polaris_control` under the new folder tree without change to functionality.

### Added

**`polaris_planning/`** — new sub-package

- `src/path_from_points.cpp`: generates a smooth reference path from a list of 2D waypoints. Publishes the sampled path as a `nav_msgs/Path` on a configurable topic and exposes an `is_path_closed` service.
- `src/path_from_file.cpp`: loads a path from a text file and republishes it.
- `src/path_from_equation.cpp`: generates a path from a parametric equation defined in config.
- `config/path_from_points.yaml`, `path_from_file.yaml`, `path_from_equation.yaml`: configuration files for each planner node.
- `srv/Trigger.srv`: custom service definition used by path planners to signal open/closed path topology.
- Sample path files under `path_txt/` (`path_1.txt` – `path_4.txt`).
- `scripts/tf_fixed.py`: utility script for broadcasting a static TF for testing.

**`polaris_control/`** — new sub-package structure (code migrated from root)

- `src/feedback_linearization.py`: Python node that converts a desired global velocity vector into linear and angular velocity commands using feedback linearisation.
- `config/pioneer_params.yaml`: new Pioneer-specific parameter file.
- `config/scout_params.yaml`: new Scout Mini parameter file.
- `launch/navigation.launch.py`, `safe_nav_stack.launch.py`, `demo.launch.py`, `feedback_linearization.launch.py`: full ROS 2 Python launch files replacing the previous XML launchers.
- `urdf/model.urdf`: robot URDF used for visualisation in RViz.

### Changed

- `CMakeLists.txt`: updated to reflect new package structure, install paths, and added `polaris_planning` build rules.
- `package.xml`: updated dependencies to include `polaris_planning` requirements and corrected package name.
- `setup.py`: updated entry points for new package paths.
- Removed legacy XML launch files (`launch/demo.xml`, `launch/test_obstacle_detection.xml`) in favour of Python launch files.

---

## 2025-10-24 - Feedback Linearisation ROS 2 Compatibility Fix

### Fixed

- Rewrote `src/feedback_linearization.py` for full ROS 2 (rclpy) compatibility. Removed ~400 lines of ROS 1 code; replaced with a clean `rclpy.node.Node` implementation (~120 lines) that subscribes to velocity vectors and publishes `Twist` commands.
- Updated `CMakeLists.txt` and `package.xml` to declare the correct ROS 2 build and runtime dependencies.
- Corrected `setup.py` entry point configuration.

---

## 2025-10-19 - AMCL Pose Integration and Obstacle Avoidance Fixes

### Fixed

- `src/feedback_linearization.py`: added callback and parameter support for subscribing to `/amcl_pose` (`PoseWithCovarianceStamped`) as an alternative pose source.
- `src/vector_field_control.cpp`: extended pose handling to accept AMCL pose topic in addition to TF.
- `config/closest_obstacle_detector_params.yaml` and `launch/test_obstacle_detection.xml`: corrected topic names and frame IDs that caused the obstacle detector to receive no data.
- `src/closest_obstacle_detector.cpp`: fixed a frame-mismatch bug that caused obstacle positions to be reported in the wrong reference frame.
- `src/vector_field_control.cpp`: corrected gain signs and topic remappings in the obstacle avoidance blending logic.

---

## 2025-10-15 - LaserScan 2D Obstacle Detection with DBSCAN Clustering

### Added

- `src/closest_obstacle_detector.cpp`: new ROS 2 node that subscribes to `sensor_msgs/LaserScan`, clusters the scan points with a DBSCAN algorithm, and publishes the position of the closest detected cluster as an obstacle marker.
- `config/closest_obstacle_detector_params.yaml`: configuration file exposing DBSCAN parameters (`eps`, `min_points`), scan topic name, and output topic name.
- `launch/test_obstacle_detection.xml`: launch file to test the detector standalone.
- Updated `CMakeLists.txt` to build and install the new node.

---

## 2025-10-09 - Scout Mini Real Robot Bring-Up (FAST-LIO + AMCL)

### Changed

- `src/feedback_linearization.py`: adjusted parameters and pose handling for real Scout Mini hardware running FAST-LIO odometry and AMCL localisation.
- `src/vector_field_control.cpp`: tuned control gains for real-robot deployment.
- `launch/demo.xml`: updated node arguments to match the real-robot topic and TF frame names.

---

## 2026-04-28 - Polaris AMCL Startup and Goal Convergence Fixes

This update addresses two operational problems observed when using Polaris with AMCL:

1. The robot sometimes did not start moving after receiving a goal until it was manually nudged with teleop.
2. The robot slowed down excessively near the final goal and could continue crawling without ever satisfying a clean stop condition.

The startup problem came from Polaris consuming pose information passively. AMCL and the TF tree may not publish a fresh pose sample exactly when a goal is sent, especially when AMCL update thresholds are movement-gated. At the same time, the planner and controller initialized their internal robot pose to `(0, 0, 0)`, which meant they could plan or control from a placeholder pose before localization was actually ready.

The convergence problem came from using the final path index as the stopping condition for open paths. That is brittle because the closest sampled path point may not become the last index at the same time the physical robot is close enough to the goal. The controller now uses distance-to-goal semantics instead.

### Fixed

- Prevented the planner from generating paths from the default `(0, 0, 0)` robot pose before localization is available. The planner now tries to refresh the robot pose from TF immediately before planning and refuses to publish a path until it has a valid pose.
- Prevented the vector field controller from publishing motion commands before it has a valid robot pose. The controller now actively refreshes pose from TF each control cycle and publishes zero vector commands while localization is unavailable.
- Added support for the correctly named `PoseWithCovarianceStamped` pose mode while keeping compatibility with the existing misspelled `PoseWithCovarience` value.
- Fixed near-goal behavior for open paths. The controller now uses a distance-based goal tolerance with hysteresis instead of relying on the closest path index becoming exactly the final path point.
- Added smooth slowdown near the final goal with a configurable minimum tracking speed before the stop tolerance. This prevents the robot from crawling indefinitely just outside the goal while still allowing it to stop cleanly once inside tolerance.
- Corrected the feedback linearization loop timing by adding a configurable `control_period` parameter. The default is now `0.05 s` instead of the previous effective `0.5 s` loop.
- Loaded the existing `path_from_points.yaml` file in Polaris navigation launch files so planner frame parameters are applied during normal launches.

### How It Works

- `path_from_points` keeps a `has_robot_pose` gate. On goal or start-service planning requests, it first asks TF for the latest transform between the configured global frame and robot frame. If TF cannot provide it yet, planning is skipped instead of publishing a path from a fake origin pose.
- `vector_field_control` keeps a `has_pose` gate. The control loop refreshes the robot pose from TF before computing commands. Until a valid pose exists, it publishes a zero vector to the feedback linearization node.
- The controller latches goal completion for open paths once the robot is inside `goal_tolerance`. The latch uses `goal_hysteresis` so small localization noise near the threshold does not repeatedly restart motion.
- The controller scales the vector field speed inside `slowdown_radius` using a smoothstep profile. Outside the final tolerance, the command keeps at least `min_tracking_speed`, avoiding the asymptotic low-speed behavior that made the robot crawl forever.
- Feedback linearization remains responsible only for converting the desired global vector into linear and angular velocity commands. Goal semantics stay in the vector field controller, where path and goal context are available.

### Added Parameters

- `goal_tolerance`: distance threshold for declaring an open-path goal reached.
- `goal_hysteresis`: distance threshold used to keep the reached-goal latch stable.
- `slowdown_radius`: distance from the final goal where speed scheduling starts.
- `min_tracking_speed`: minimum vector speed used outside the goal tolerance.
- `control_period`: feedback linearization loop period.
