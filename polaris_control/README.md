# Espeleo Control 2 - ROS 2 Vector Field Controller

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![License](https://img.shields.io/badge/License-MIT-green)

Robot controller package based on vector fields. It is designed to guide a robot along a reference path sent by the [planner](https://github.com/ITVRoC/polaris_planning) while reactively avoiding obstacles. The package also includes a simple simulation node to test and demonstrate the controller.

The obstacle avoidance implementation is based on the paper: [Vector field for curve tracking with obstacle avoidance](https://ieeexplore.ieee.org/document/9992435) by Nunes, A. H. D., et al. (2022).

## Installation

### Prerequisites
- ROS 2 Jazzy (or current distribution)
- `colcon`
- A functional ROS 2 workspace

### Installation Steps

1.  Navigate to the `src` folder of your ROS 2 workspace, or if you don't have a workspace, create one:
    ```bash
    mkdir -p ~/espeleo_ws/src
    cd ~/espeleo_ws/src
    ```

2.  Clone this repository:
    ```bash
    git clone [https://github.com/ITVRoC/polaris_control.git](https://github.com/ITVRoC/polaris_control.git)
    ```
3. Install dependencies
    ```bash
    cd ~/espeleo_ws
    rosdep install -i --from-paths src/polaris_control --rosdistro $ROS_DISTRO -y
    ```
    
4.  Build the package:
    ```bash
    cd ~/espeleo_ws
    colcon build --symlink-install
    ```
    
5.  Source the workspace:
    ```bash
    source install/setup.bash
    ```

## How to Use

To run a full demonstration including the controller, path planner (you must have the [planner](https://github.com/ITVRoC/polaris_planning) compiled in the same workspace), robot simulator, and RViz2, use the provided launch file:

```bash
ros2 launch polaris_control demo.xml
```

This will start all the necessary nodes, and you will be able to see the simulated robot (red sphere) following a path (white spheres) and avoiding an obstacle (blue sphere).

## Package Nodes

This package contains two main nodes:

### 1. `vector_field_controller`

This is the main control node. It generates velocity commands (`geometry_msgs/msg/Vector3`) to make the robot follow a path and avoid obstacles.

#### Subscribed Topics
- **`path_topic_name`** (`nav_msgs/msg/Path`, default: `ref_path`): The reference path for the robot to follow.
- **`pose_topic_name`** (`tf2_msgs/msg/TFMessage` or `nav_msgs/msg/Odometry`, default: `tf`): The robot's current pose. The type is configurable via the `pose_topic_type` parameter.
- **`closest_obstacle_topic_name`** (`geometry_msgs/msg/Point`, default: `closest_obstacle`): The position of the nearest obstacle.

#### Published Topics
- **`cmd_vel_topic_name`** (`geometry_msgs/msg/Vector3`, default: `cmd_vel`): The calculated velocity command for the robot.
- **`/visualization_command`** (`visualization_msgs/msg/Marker`): Visualizes the current field vector as an arrow in RViz, originating from the robot's position.

#### Services Called
- **`is_path_closed_service_name`** (`std_srvs/srv/Trigger`, default: `is_path_closed`): Queries a service (usually provided by the planning node) to determine if the reference path is closed or open.

#### Parameters
- `speed_ref` (double, default: 0.5): The robot's reference speed in m/s.
- `convergence_gain` (double, default: 5.0): Gain that adjusts how quickly the robot converges to the path.
- `flag_follow_obstacle` (bool, default: `true`): **Main flag to enable or disable the obstacle avoidance logic.**
- `lambda` (double, default: 0.4): The desired distance to circumnavigate an obstacle (in meters).
- `switch_dist` (double, default: 0.55): Activation distance (`D_in` in the paper). Below this distance, the obstacle avoidance field is fully activated.
- `switch_dist_outer` (double, default: 0.7): Transition distance (`D_in^0` in the paper). Between `switch_dist` and `switch_dist_outer`, the field is a smooth blend of the path-following and avoidance fields.

---

### 2. `robot_simulator`

This is a legacy standalone Python node that simulates a robot and an obstacle
for testing and demonstration purposes. Its configurable topic names are
relative, but its TF publisher is not remapped by a robot-aware launch.

#### Subscribed Topics
- **`input_topic`** (`geometry_msgs/msg/Vector3`, default: `cmd_vel`): Receives velocity commands to move the simulated robot.

#### Published Topics
- **`/tf`** (`tf2_msgs/msg/TFMessage`): Publishes the robot's coordinate transformation (from `parent_frame_id` to `child_frame_id`).
- **`marker_topic`** (`visualization_msgs/msg/Marker`, default: `ball_marker`): Publishes a sphere marker to visualize the robot's position in RViz.
- **`closest_obstacle_topic`** (`geometry_msgs/msg/Point`, default: `closest_obstacle`): Publishes the position of a simulated static obstacle.
- **`visual_obstacle_topic`** (`visualization_msgs/msg/Marker`, default: `visual_obstacle`): Publishes a sphere marker to visualize the obstacle in RViz.

#### Parameters
- `parent_frame_id` (string, default: `map`): The parent reference frame for the robot's TF.
- `child_frame_id` (string, default: `base_link`): The child reference frame for the robot's TF.
- `ball_radius` (double, default: 0.1): The radius of the robot's visual marker.
- `obstacle_pos` (double[], default: `[2.1, 2.1, 0.0]`): The [x, y, z] position of the static obstacle.
- `obstacle_radius` (double, default: 0.15): The radius of the obstacle's visual marker.


## Package Nodes (TODO)

Arquivos de referência com alguns métodos já implementados estão em polaris_control/src/TODO

### 3. `Closest Obstacle`

Deve receber diferentes tipos de sensores lidar de entrada e retornar o obstáculo mais próximo da posição atual do robô de modo compatível com o `vector_field_controller`. Use [DBScan clustering](https://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html).

### 4. `Feedback Linearization`

Recebe o comando de vetor velocidade e retorna um comando compatível com robô diferencial (v e omega).

### 5. `Follow corridor` (AVANÇADO)

Deve tratar a entrada de lidar reativamente para movimentar o robô de modo equidistante às paredes.

### 5. `Follow wall` (AVANÇADO)

Deve tratar a entrada de lidar reativamente para movimentar o robô de modo a seguir uma parede a uma distância constante dela.

## Troubleshooting

If you run into the following error while building:

    ```bash
        fatal error: tf2_geometry_msgs/tf2_geometry_msgs.hpp: No such file or directory
        14 | #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
    ```

Change from .hpp to .h and try again.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contact

Thales Andrade Soares
<thalesasoares02@gmail.com>
Instituto Tecnológico Vale (ITV)
Universidade Federal de Minas Gerais (UFMG)
