cmake_minimum_required(VERSION 3.11)
project(generic_subscriber)
message(STATUS "CMake running for project: \"${PROJECT_NAME}\"")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -fPIC -Wp,-D_GLIBCXX_ASSERTIONS")
# Include directories
include_directories(include)


add_library(generic_subscriber STATIC src/generic_subscriber.cpp)
ament_target_dependencies(generic_subscriber rclcpp dynmsg)
target_include_directories(generic_subscriber PUBLIC ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)