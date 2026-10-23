cmake_minimum_required(VERSION 3.8)
project(ros2_control_crospi)

message("--------------ros2_control_crospi compiling ----------------")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(crospi_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Required to resolve the link interface of crospi_core::crospi_utils (string_interpolate and JsonChecker)
find_package(crospi_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(etasl REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Boost 1.89 CONFIG REQUIRED)
message(STATUS "Using Boost version ${Boost_VERSION}")
message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")


# ----------------------------------------------------------------------------------------------
# Plugin library
# ----------------------------------------------------------------------------------------------
set(src_ros2_control_crospi
    src/ros2_control_crospi.cpp
    src/robot_description_loader.cpp
)

add_library(ros2_control_crospi
            SHARED
            ${src_ros2_control_crospi})


# All commented dependencies are already inherited and linked thanks to crospi_core::robot_interfacing_utils (thanks to ament_export_dependencies)
set(rosdependencies
  pluginlib
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  hardware_interface
  ament_index_cpp
)

set(dependencies
    crospi_core::robot_interfacing_utils
    crospi_core::crospi_utils   # string_interpolate ($[package] paths) and JsonChecker
  )

set(ros_targets
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${lifecycle_msgs_TARGETS}
    hardware_interface::hardware_interface
    ament_index_cpp::ament_index_cpp
  )


pluginlib_export_plugin_description_file(crospi_core plugins.xml)


target_include_directories(ros2_control_crospi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})


# ----------------------------------------------------------------------------------------------
# Test executable: loads the driver plugin through pluginlib (exactly as the crospi_node does)
# and drives it with a velocity profile, without needing the full crospi_node.
# ----------------------------------------------------------------------------------------------
add_executable(test_ros2_control_crospi src/test_ros2_control_crospi.cpp)
target_include_directories(test_ros2_control_crospi PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${Boost_INCLUDE_DIRS})


#The following is done because ROS2 deprecated ament_target_dependencies in favor of target_link_libraries, but the former is still required for older (legacy) ROS2 versions.
if(DEFINED ENV{ROS_DISTRO} AND (
  "$ENV{ROS_DISTRO}" STREQUAL "foxy" OR
  "$ENV{ROS_DISTRO}" STREQUAL "galactic" OR
  "$ENV{ROS_DISTRO}" STREQUAL "humble" OR
  "$ENV{ROS_DISTRO}" STREQUAL "iron" OR
  "$ENV{ROS_DISTRO}" STREQUAL "jazzy"
))
      ament_target_dependencies(ros2_control_crospi ${rosdependencies})
      target_link_libraries(ros2_control_crospi ${dependencies})
      ament_target_dependencies(test_ros2_control_crospi pluginlib rclcpp)
      target_link_libraries(test_ros2_control_crospi crospi_core::robot_interfacing_utils crospi_core::crospi_utils)
      message(STATUS "Legacy ROS_DISTRO is $ENV{ROS_DISTRO}, using ament_target_dependencies")
else()
    target_link_libraries(ros2_control_crospi  ${dependencies} ${ros_targets})
    target_link_libraries(test_ros2_control_crospi crospi_core::robot_interfacing_utils crospi_core::crospi_utils
                          pluginlib::pluginlib rclcpp::rclcpp)
    message(STATUS "Modern ROS_DISTRO detected ($ENV{ROS_DISTRO}), using target_link_libraries")
endif()


ament_export_dependencies(crospi_core)

ament_export_libraries(
  ros2_control_crospi
)

install(
  TARGETS ros2_control_crospi
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS test_ros2_control_crospi
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY json_schemas config
  DESTINATION share/${PROJECT_NAME})

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
