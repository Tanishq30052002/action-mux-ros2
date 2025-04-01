cmake_minimum_required(VERSION 3.8)
project(generic_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_subdirectory(generic_subscriber)

add_executable(run_generic_subscriber src/run_generic_subscriber.cpp)
target_link_libraries(run_generic_subscriber PRIVATE generic_subscriber)

install(TARGETS
  run_generic_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Export package dependencies
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
