cmake_minimum_required(VERSION 3.8)
project(robograsp_vision_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robograsp_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# piper_control_interface library
add_library(piper_control_interface SHARED
  src/piper_control_interface.cpp
)
target_include_directories(piper_control_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(piper_control_interface rclcpp robograsp_interfaces std_msgs)

# bridge_node executable
add_executable(bridge_node
  src/bridge_node.cpp
)
target_include_directories(bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(bridge_node piper_control_interface)
ament_target_dependencies(bridge_node rclcpp robograsp_interfaces std_msgs)

install(TARGETS
  piper_control_interface
  bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
