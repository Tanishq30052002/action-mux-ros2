cmake_minimum_required(VERSION 3.8)
project(action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # Ensure action generation

# Generate ROS 2 action interfaces
rosidl_generate_interfaces(${PROJECT_NAME} "action/MoveToPose.action" DEPENDENCIES std_msgs)

# Build action server
include_directories(include )
add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action geometry_msgs action_tutorials_interfaces)

rosidl_get_typesupport_target(action_server ${PROJECT_NAME} rosidl_typesupport_cpp)
# Fix: Use only plain signature
target_link_libraries(action_server ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Build action client
add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action geometry_msgs action_tutorials_interfaces)

rosidl_get_typesupport_target(action_client ${PROJECT_NAME} rosidl_typesupport_cpp)
# Fix: Use only plain signature
target_link_libraries(action_client ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Build goal publisher
add_executable(goal_publisher_3s src/goal_publisher_3s.cpp)
ament_target_dependencies(goal_publisher_3s rclcpp geometry_msgs)

# Build goal publisher
add_executable(goal_publisher_5s src/goal_publisher_5s.cpp)
ament_target_dependencies(goal_publisher_5s rclcpp geometry_msgs)

# Build goal publisher
add_executable(goal_publisher_7s src/goal_publisher_7s.cpp)
ament_target_dependencies(goal_publisher_7s rclcpp geometry_msgs)

install(TARGETS action_server action_client goal_publisher_3s goal_publisher_5s goal_publisher_7s DESTINATION lib/${PROJECT_NAME})

# Export package dependencies
ament_export_dependencies(rclcpp rclcpp_action geometry_msgs action_tutorials_interfaces)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  # Move inside testing block
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize package
ament_package()
