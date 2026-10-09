# Built and tested with colcon on ROS2 Jazzy (WSL2 Ubuntu 24.04, ros2_control 4.48), mock backend only.
cmake_minimum_required(VERSION 3.16)
project(episode_ros2_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

add_library(episode_ros2_control SHARED episode_system_hardware.cpp)
target_include_directories(episode_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/episode_ros2_control>
)
ament_target_dependencies(episode_ros2_control PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface episode_system_hardware.xml)

install(FILES episode_system_hardware.hpp DESTINATION include/episode_ros2_control)
install(TARGETS episode_ros2_control
  EXPORT export_episode_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES episode_ros2_control.xacro DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY urdf launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_episode_system_hardware test/test_episode_system_hardware.cpp)
  target_link_libraries(test_episode_system_hardware episode_ros2_control)
  ament_target_dependencies(test_episode_system_hardware ${THIS_PACKAGE_INCLUDE_DEPENDS})
endif()

ament_export_targets(export_episode_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
