cmake_minimum_required(VERSION 3.8)
project(robograsp_vision_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robograsp_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# camera_detector library
add_library(camera_detector SHARED
  src/camera_detector.cpp
  src/depth_geometry_detector.cpp
)
target_include_directories(camera_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camera_detector rclcpp sensor_msgs cv_bridge OpenCV)

# perception_node executable
add_executable(perception_node
  src/perception_node.cpp
)
target_include_directories(perception_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(perception_node camera_detector)
ament_target_dependencies(perception_node rclcpp robograsp_interfaces geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)

# install targets
install(TARGETS camera_detector
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS perception_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(camera_detector)

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
