cmake_minimum_required(VERSION 3.11)
project(calculator)
message(STATUS "CMake running for project: \"${PROJECT_NAME}\"")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -fPIC -Wp,-D_GLIBCXX_ASSERTIONS")
# Include directories
include_directories(include)
include_directories(${calculate_action_pkg_INCLUDE_DIRS})
message(STATUS "${calculate_action_pkg_INCLUDE_DIRS}")


add_library(calculator_client STATIC src/calculator_client.cpp)
ament_target_dependencies(calculator_client rclcpp rclcpp_action geometry_msgs calculator_msgs)
target_include_directories(calculator_client PUBLIC ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

add_library(calculator_server STATIC src/calculator_server.cpp)
ament_target_dependencies(calculator_server rclcpp rclcpp_action geometry_msgs calculator_msgs)
target_include_directories(calculator_server PUBLIC ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
