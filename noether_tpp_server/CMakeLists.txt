cmake_minimum_required(VERSION 3.8)

project(noether_tpp_server LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(noether_gui REQUIRED)
find_package(boost_plugin_loader REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanToolPath.srv"
)

add_executable(tool_path_planner_server src/tool_path_planner_server.cpp)
ament_target_dependencies(tool_path_planner_server
  rclcpp
  yaml-cpp
  noether_gui
  boost_plugin_loader
)

rosidl_target_interfaces(tool_path_planner_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS tool_path_planner_server
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
