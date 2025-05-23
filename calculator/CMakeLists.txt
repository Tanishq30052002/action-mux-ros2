cmake_minimum_required(VERSION 3.8)
project(calculator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(calculator_msgs REQUIRED)

add_subdirectory(calculator)

add_executable(run_client_calculate src/run_client_calculate.cpp)
target_link_libraries(run_client_calculate PRIVATE calculator_client)

add_executable(run_server_calculate src/run_server_calculate.cpp)
target_link_libraries(run_server_calculate PRIVATE calculator_server)

add_executable(run_calculator_goal_publisher src/run_calculator_goal_publisher.cpp)
target_link_libraries(run_calculator_goal_publisher PRIVATE calculator_goal_publisher)

install(TARGETS
  run_client_calculate
  run_server_calculate
  run_calculator_goal_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Export package dependencies
ament_export_dependencies(rclcpp rclcpp_action)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()