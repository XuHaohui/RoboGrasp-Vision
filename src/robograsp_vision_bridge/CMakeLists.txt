cmake_minimum_required(VERSION 3.8)
project(robograsp_vision_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robograsp_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# piper_control_interface library
add_library(piper_control_interface SHARED
  src/piper_control_interface.cpp
)
target_include_directories(piper_control_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(piper_control_interface rclcpp geometry_msgs robograsp_interfaces tf2 tf2_ros tf2_geometry_msgs)

# bridge_node executable
add_executable(bridge_node
  src/bridge_node.cpp
)
target_include_directories(bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(bridge_node piper_control_interface)
ament_target_dependencies(bridge_node rclcpp robograsp_interfaces geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)

# install targets
install(TARGETS
  piper_control_interface
  bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# install headers
install(DIRECTORY include/
  DESTINATION include
)

# install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
