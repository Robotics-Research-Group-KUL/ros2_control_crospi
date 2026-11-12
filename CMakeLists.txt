cmake_minimum_required(VERSION 3.8)
project(ros2_control_crospi)

message("--------------ros2_control_crospi compiling ----------------")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
# crospi_core exports all its dependencies (rclcpp, rclcpp_lifecycle, etasl, pluginlib, ament_index_cpp, ...), so
# they are found and linked through crospi_core::crospi_core. Only the extra ros2_control dependencies are found here.
find_package(crospi_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)

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


target_link_libraries(ros2_control_crospi PUBLIC
  crospi_core::crospi_core
  hardware_interface::hardware_interface
  ${lifecycle_msgs_TARGETS})


pluginlib_export_plugin_description_file(crospi_core plugins.xml)


target_include_directories(ros2_control_crospi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# ----------------------------------------------------------------------------------------------
# Test executable: loads the driver plugin through pluginlib (exactly as the crospi_node does)
# and drives it with a velocity profile, without needing the full crospi_node.
# ----------------------------------------------------------------------------------------------
add_executable(test_ros2_control_crospi src/test_ros2_control_crospi.cpp)
target_include_directories(test_ros2_control_crospi PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)


target_link_libraries(test_ros2_control_crospi crospi_core::crospi_core)


ament_export_dependencies(crospi_core)

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
