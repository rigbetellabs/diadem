cmake_minimum_required(VERSION 3.8)
project(diadem_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(PROGRAMS
  scripts/robot_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(goal_status_publisher src/goal_status_publisher.cpp)
ament_target_dependencies(goal_status_publisher rclcpp std_msgs action_msgs)

add_executable(robot_pose_publisher src/robot_pose.cpp)
ament_target_dependencies(robot_pose_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(nav2_goal_canceller src/cancle_nav.cpp)
ament_target_dependencies(
  nav2_goal_canceller
  rclcpp
  rclcpp_action
  nav2_msgs
  std_msgs
  action_msgs
)

install(TARGETS
  goal_status_publisher
  robot_pose_publisher
  nav2_goal_canceller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch scripts src
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
