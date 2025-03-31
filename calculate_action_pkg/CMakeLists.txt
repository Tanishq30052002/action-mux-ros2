cmake_minimum_required(VERSION 3.8)
project(calculate_action_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(calculator_msgs REQUIRED)


add_subdirectory(calculator)

include_directories(include)
add_executable(run_client_calculate src/run_client_calculate.cpp)
target_link_libraries(run_client_calculate PRIVATE calculator_client)
add_executable(run_server_calculate src/run_server_calculate.cpp)
target_link_libraries(run_server_calculate PRIVATE calculator_server)

install(TARGETS run_client_calculate run_server_calculate DESTINATION lib/${PROJECT_NAME})

# Export package dependencies
ament_export_dependencies(rclcpp rclcpp_action geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()