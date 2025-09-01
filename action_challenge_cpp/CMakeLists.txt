cmake_minimum_required(VERSION 3.8)
project(action_challenge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(move_one_axis_server src/move_one_axis_server.cpp)
ament_target_dependencies(move_one_axis_server rclcpp rclcpp_action my_robot_interfaces)

add_executable(move_one_axis_client src/move_one_axis_client.cpp)
ament_target_dependencies(move_one_axis_client rclcpp rclcpp_action my_robot_interfaces std_msgs)

add_executable(move_one_axis_server_lifecycle src/move_one_axis_server_lifecycle.cpp)
ament_target_dependencies(move_one_axis_server_lifecycle rclcpp rclcpp_action my_robot_interfaces rclcpp_lifecycle)

install(TARGETS
  move_one_axis_server
  move_one_axis_client
  move_one_axis_server_lifecycle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
