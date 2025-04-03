cmake_minimum_required(VERSION 3.8)

project(qrb_ros_audio_common)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(ros_audio_common_component SHARED src/audio_common.cpp)

target_link_libraries(ros_audio_common_component qrb_audio_common_lib)

rclcpp_components_register_node(
  ros_audio_common_component
  PLUGIN "qrb_ros::audio_common::AudioCommonNode"
  EXECUTABLE audio_common_node_exec
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
