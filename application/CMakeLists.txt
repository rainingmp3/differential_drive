
cmake_minimum_required(VERSION 3.5)
project(application)

# ---- Dependencies ----
# ament_cmake is required for ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- Library (reusable code) ----
# Put reusable code (PID, potential field, utils) into a static library
add_library(control_lib
  src/control/pid.cpp
  src/control/wrap_angle.cpp
  src/control/lqr.cpp
)

# Link ROS2 dependencies to the library
ament_target_dependencies(control_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# Add include directories for the library
target_include_directories(control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>   # headers during build
  $<INSTALL_INTERFACE:include>                             # headers after install
  ${Eigen3_INCLUDE_DIRS}                                   # Eigen headers
)

# Link Eigen into the library
target_link_libraries(control_lib Eigen3::Eigen)

# ---- Executable (entry point) ----
# Only controller.cpp here!
add_executable(controller
  src/control/controller.cpp
)

# Link the library into the executable
target_link_libraries(controller control_lib)

# Link ROS2 dependencies (needed for rclcpp::Node in controller.cpp)
ament_target_dependencies(controller
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# ---- Install rules ----
# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library + executable
install(TARGETS
  control_lib
  controller
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Testing (optional) ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ---- Finalize package ----
ament_package()

