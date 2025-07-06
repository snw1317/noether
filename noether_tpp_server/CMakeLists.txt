cmake_minimum_required(VERSION 3.5.1)

find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(noether_gui REQUIRED)

# generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanToolPath.srv"
)

add_executable(${PROJECT_NAME}_node src/tool_path_planner_server.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC rclcpp::rclcpp yaml-cpp noether::noether_gui)

# Set C++ standard and other compile options
target_cxx_version(${PROJECT_NAME}_node PUBLIC VERSION 14)
target_clang_tidy(${PROJECT_NAME}_node ENABLE ${NOETHER_ENABLE_CLANG_TIDY} WARNINGS_AS_ERRORS ${NOETHER_ENABLE_TESTING} CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

configure_package(NAMESPACE noether TARGETS ${PROJECT_NAME}_node DEPENDENCIES rclcpp rosidl_default_runtime yaml-cpp noether_gui)
