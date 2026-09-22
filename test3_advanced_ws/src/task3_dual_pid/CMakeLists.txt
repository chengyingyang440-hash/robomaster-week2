cmake_minimum_required(VERSION 3.8)
project(task3_dual_pid)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(motor_simulator src/motor_simulator.cpp)
add_executable(angle_controller src/angle_controller.cpp)
add_executable(speed_controller src/speed_controller.cpp)

ament_target_dependencies(motor_simulator rclcpp std_msgs)
ament_target_dependencies(angle_controller rclcpp std_msgs)
ament_target_dependencies(speed_controller rclcpp std_msgs)

install(
  TARGETS motor_simulator angle_controller speed_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
