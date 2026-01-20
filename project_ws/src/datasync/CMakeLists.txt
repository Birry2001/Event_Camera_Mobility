cmake_minimum_required(VERSION 3.8)
project(datasync)

# ---  Compiler Configuration ---
# Set C++ Standard to C++17 (Recommended for ROS 2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---  Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(dvs_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# ---  Build Executable ---
add_executable(Motion_Compensation src/Motion_Compensation.cpp)

# ---  Include Directories ---
target_include_directories(Motion_Compensation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# ---  Link Libraries & Dependencies ---
# Link ROS 2 packages
ament_target_dependencies(Motion_Compensation
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  dvs_msgs
)

# Link System libraries (OpenCV)
target_link_libraries(Motion_Compensation ${OpenCV_LIBS})

# ---  Installation ---
# Install the executable
install(TARGETS
  Motion_Compensation
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory (Optional allows build to succeed even if folder is empty)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL 
)

# ---  Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
