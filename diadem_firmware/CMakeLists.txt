cmake_minimum_required(VERSION 3.8)
project(diadem_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})

add_executable(network_status_publisher src/network_status_publisher.cpp)
ament_target_dependencies(network_status_publisher rclcpp std_msgs)

add_executable(goal_status_publisher src/goal_status_publisher.cpp)
ament_target_dependencies(goal_status_publisher rclcpp std_msgs action_msgs)
 

install(PROGRAMS
   #compiled_scripts/pixhawk_to_cmd.pyc compiled_scripts/robot_diagnostics.pyc compiled_scripts/rbl_logger.pyc
   scripts/pixhawk_to_cmd.py
   scripts/imu_read_pub.py
   DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS
  goal_status_publisher
  network_status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch scripts src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
