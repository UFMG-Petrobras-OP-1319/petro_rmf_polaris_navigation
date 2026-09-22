cmake_minimum_required(VERSION 3.8)
project(polaris_control)


#-----------------------------------------------------------------------------
# 1. Compiler Configuration
#-----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#-----------------------------------------------------------------------------
# 2. Dependency Management
#-----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)

# Core ROS2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Eigen3 configuration
# find_package(eigen3_cmake_module REQUIRED)
# find_package(Eigen3 REQUIRED)
# message(STATUS "Found Eigen3 v${EIGEN3_VERSION}")

# # Eigen compiler settings (applied globally)
# add_compile_definitions(EIGEN_MPL2_ONLY EIGEN_NO_DEBUG)

#-----------------------------------------------------------------------------
# 3. Executable Targets
#-----------------------------------------------------------------------------
# Define executables with their specific dependencies

# Define the executable
add_executable(vector_field_controller src/vector_field_control.cpp)
ament_target_dependencies(vector_field_controller
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)
target_compile_features(vector_field_controller PRIVATE cxx_std_17)

# Closest Obstacle Detector (LaserScan 2D)
add_executable(closest_obstacle_detector src/closest_obstacle_detector.cpp)
ament_target_dependencies(closest_obstacle_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_compile_features(closest_obstacle_detector PRIVATE cxx_std_17)

# add_executable(follow_wall src/follow_wall.cpp)
# ament_target_dependencies(follow_wall
#   rclcpp
#   std_msgs
#   std_srvs
#   nav_msgs
#   geometry_msgs
#   sensor_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   visualization_msgs
# )
# target_compile_features(follow_wall PRIVATE cxx_std_17)


# add_executable(follow_corridor src/follow_corridor.cpp)
# ament_target_dependencies(follow_corridor
#   rclcpp
#   std_msgs
#   std_srvs
#   nav_msgs
#   geometry_msgs
#   sensor_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   visualization_msgs
# )
# target_compile_features(follow_corridor PRIVATE cxx_std_17)

#-----------------------------------------------------------------------------
# 4. Installation Configuration
#-----------------------------------------------------------------------------
# Install executables
install(TARGETS 
  vector_field_controller
  closest_obstacle_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/robot_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/feedback_linearization.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/follower_control.py
  src/FollowerControlState.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, config, URDF, and meshes (RViz model) for navigation.launch.py
install(
  DIRECTORY 
    launch 
    urdf
    config
  DESTINATION share/${PROJECT_NAME}
)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
else()
  message(WARNING "polaris_control: 'meshes' directory missing — copy scout_description/meshes here for RViz visuals, then rebuild.")
endif()



#-----------------------------------------------------------------------------
# 5. Package Export and Testing
#-----------------------------------------------------------------------------
# Export dependencies for downstream packages
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  eigen3_cmake_module
)

# Enable linting if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

