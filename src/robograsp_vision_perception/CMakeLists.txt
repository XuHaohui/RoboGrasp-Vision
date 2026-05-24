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

# mock_detector library
add_library(mock_detector SHARED
  src/mock_detector.cpp
)
target_include_directories(mock_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mock_detector rclcpp geometry_msgs)

# camera_detector library
add_library(camera_detector SHARED
  src/camera_detector.cpp
)
target_include_directories(camera_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camera_detector rclcpp sensor_msgs cv_bridge OpenCV)

# mock_perception executable
add_executable(mock_perception
  src/mock_perception_node.cpp
)
target_include_directories(mock_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mock_perception mock_detector)
ament_target_dependencies(mock_perception rclcpp robograsp_interfaces geometry_msgs std_msgs)

# perception_node executable
add_executable(perception_node
  src/perception_node.cpp
)
target_include_directories(perception_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(perception_node mock_detector camera_detector)
ament_target_dependencies(perception_node rclcpp robograsp_interfaces geometry_msgs std_msgs)

# install targets
install(TARGETS
  mock_detector
  camera_detector
  mock_perception
  perception_node
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
