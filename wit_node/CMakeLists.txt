cmake_minimum_required(VERSION 3.8)
project(wit_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

# Add the C library
add_library(mylib src/wit_c_sdk.c)
target_include_directories(mylib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(imu_pub src/imu_publisher.cpp)
ament_target_dependencies(imu_pub rclcpp std_msgs sensor_msgs)
target_link_libraries(imu_pub mylib)

install(TARGETS
  imu_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
