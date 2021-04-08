cmake_minimum_required(VERSION 3.5)
project(ackermann_steering_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)


add_library(ackermann_steering_controller SHARED
  src/ackermann_steering_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)


target_include_directories(ackermann_steering_controller PRIVATE include)
ament_target_dependencies(ackermann_steering_controller
  builtin_interfaces
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ackermann_steering_controller PRIVATE "ACKERMANN_STEERING_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(ackermann_steering_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface ackermann_steering_plugin.xml)

###########
# Install #
###########

install(
  DIRECTORY include/
  DESTINATION include
  )

install(
  TARGETS ackermann_steering_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ackermann_steering_controller
)

ament_package()
