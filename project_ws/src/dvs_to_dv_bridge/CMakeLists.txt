cmake_minimum_required(VERSION 3.8)
project(dvs_to_dv_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dvs_msgs REQUIRED)
find_package(dv_ros2_msgs REQUIRED)

add_executable(dvs_to_dv_bridge_node src/dvs_to_dv_bridge.cpp)
ament_target_dependencies(dvs_to_dv_bridge_node rclcpp dvs_msgs dv_ros2_msgs)

install(TARGETS dvs_to_dv_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
