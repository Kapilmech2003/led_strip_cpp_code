cmake_minimum_required(VERSION 3.8)
project(led_strip_cpp_code)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#add executables

add_executable(ros_code src/ros_code.cpp)
ament_target_dependencies(ros_code rclcpp std_msgs)

#install targets 

install(TARGETS
  ros_code
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
