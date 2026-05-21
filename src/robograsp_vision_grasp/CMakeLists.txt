cmake_minimum_required(VERSION 3.8)
project(robograsp_vision_grasp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robograsp_interfaces REQUIRED)

# generator library
add_library(generator SHARED
  src/generator.cpp
)
target_include_directories(generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(generator geometry_msgs robograsp_interfaces)

# grasp_node executable
add_executable(grasp_node
  src/grasp_node.cpp
)
target_include_directories(grasp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(grasp_node generator)
ament_target_dependencies(grasp_node rclcpp robograsp_interfaces geometry_msgs std_msgs)

# install targets
install(TARGETS
  generator
  grasp_node
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
