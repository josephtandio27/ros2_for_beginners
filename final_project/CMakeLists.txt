cmake_minimum_required(VERSION 3.8)
project(final_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(turtlesim_manager_component SHARED src/turtlesim_manager.cpp)
ament_target_dependencies(turtlesim_manager_component rclcpp rclcpp_action rclcpp_lifecycle rclcpp_components my_robot_interfaces turtlesim geometry_msgs)
rclcpp_components_register_nodes(turtlesim_manager_component "final_project::TurtlesimManager")

add_executable(move_turtle_action_server src/move_turtle_action_server.cpp)
ament_target_dependencies(move_turtle_action_server rclcpp rclcpp_action my_robot_interfaces turtlesim geometry_msgs)

install(TARGETS
  turtlesim_manager_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  move_turtle_action_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
