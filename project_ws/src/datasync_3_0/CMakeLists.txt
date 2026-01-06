cmake_minimum_required(VERSION 3.8)
project(datasync_3_0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dv_ros2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(motion_compensation_node src/Motion_Compensation.cpp)

target_include_directories(motion_compensation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(motion_compensation_node
  ${OpenCV_LIBS}
)

ament_target_dependencies(motion_compensation_node
  rclcpp
  sensor_msgs
  dv_ros2_msgs
  cv_bridge
)

install(TARGETS motion_compensation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
