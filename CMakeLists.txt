cmake_minimum_required(VERSION 3.8)
project(ros2_control_crospi)

message("--------------ros2_control_crospi compiling ----------------")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(crospi_core REQUIRED)

find_package(Boost 1.89 CONFIG REQUIRED)
message(STATUS "Using Boost version ${Boost_VERSION}")
message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")

set(CMAKE_CXX_STANDARD 17)


set(src_ros2_control_crospi
    src/ros2_control_crospi.cpp
)

add_library(ros2_control_crospi 
            SHARED
            ${src_ros2_control_crospi})

set(MY_EXTERNAL_LIB "") #sets it empty such that it can be redefined later to include external lib


# UNCOMENT THE FOLLOWING LINES AND MODIFY IF AN EXTERNAL LIBRARY IS DESIRED TO BE INCLUDED WITHIN THIS PACKAGE AND COMPILED BEFORE THE REST OF THE PACKAGE AS A DEPENDENCY
# set(MY_EXTERNAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/my_external_lib/)
# set(MY_EXTERNAL_LIB_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/external/my_external_lib/include)
# set(MY_EXTERNAL_LIB_DIR_SO_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/external/my_external_lib/lib)

# message("--------------MY_EXTERNAL_LIB_DIR: ${MY_EXTERNAL_LIB_DIR}----------------")

# add_custom_target(external_compiled_lib COMMAND make clean && make
#                    WORKING_DIRECTORY ${MY_EXTERNAL_LIB_DIR}/build/GNUMake
# )


# include_directories(${MY_EXTERNAL_LIB_INCLUDE})

# add_dependencies(ros2_control_crospi external_compiled_lib) #Makes sure to compile first external_compiled_lib before the ros2_control_crospi

# message("-------------- MY_EXTERNAL_LIB_DIR compiled----------------")


# find_library(MY_EXTERNAL_LIB
#              NAMES <name_of_the_so_lib>.so
#              PATHS ${MY_EXTERNAL_LIB_DIR_SO_LOCATION}
# )

# message("--------------MY_EXTERNAL_LIB: ${MY_EXTERNAL_LIB}----------------")




# set(etasl_task_utils_DIR /workspaces/devcontainer/build/crospi_core/src/misc/etasl_task_utils)

# find_library(etasl_task_utils
#              NAMES libetasl_task_utils.so
#              PATHS ${etasl_task_utils_DIR}
# )
# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/path/to/etasl_task_utils")


# find_library(etasl_task_utils
#               NAMES libetasl_task_utils.so)

# find_library(robot_interfacing_utils
#               NAMES librobot_interfacing_utils.so)

# find_library(crospi_node_utils
#               NAMES libcrospi_node_utils.so)
# find_package(crospi_node_utils REQUIRED)
# find_package(robot_interfacing_utils REQUIRED)


# All commented dependencies are already inherited and linked thanks to crospi_core::robot_interfacing_utils (thanks to ament_export_dependencies)
set(rosdependencies
  pluginlib
)

set(dependencies
    # fmt   
    # pugixml
    # jsoncpp
    # etasl::expressiongraph_context 
    # # etasl::expressiongraph_solver_qpoases 
    # etasl::expressiongraph_context_lua   
    # fmt   
    # jsoncpp
    # crospi_core::crospi_utils
    # crospi_core::crospi_node_utils
    crospi_core::robot_interfacing_utils
  )

  set(ros_targets
    pluginlib::pluginlib
    # rclcpp::rclcpp
    # rclcpp_lifecycle::rclcpp_lifecycle
    # ${std_msgs_TARGETS}
    # ${std_srvs_TARGETS}
    # ${sensor_msgs_TARGETS}
    # ${lifecycle_msgs_TARGETS}
  )
  

pluginlib_export_plugin_description_file(crospi_core plugins.xml)


target_include_directories(ros2_control_crospi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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
      message(STATUS "Legacy ROS_DISTRO is $ENV{ROS_DISTRO}, using ament_target_dependencies")
else()
    target_link_libraries(ros2_control_crospi  ${dependencies} ${ros_targets})
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

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY json_schemas
  DESTINATION share/${PROJECT_NAME})
  
# ament_export_include_directories(
#   include
# )

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
