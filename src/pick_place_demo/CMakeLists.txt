cmake_minimum_required(VERSION 3.8)
project(pick_place_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Build vision node as a component and standalone executable
add_library(vision_node SHARED
  src/vision_node.cpp
)
target_compile_definitions(vision_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(vision_node
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  message_filters
)
target_link_libraries(vision_node ${OpenCV_LIBRARIES})
rclcpp_components_register_nodes(vision_node
  "pick_place_demo::VisionNode"
)

add_executable(vision_node_exe
  src/vision_node_main.cpp
)
target_link_libraries(vision_node_exe
  vision_node
)
set_target_properties(vision_node_exe
  PROPERTIES OUTPUT_NAME vision_node
)

# Build control node as a component and standalone executable
add_library(control_node SHARED
  src/control_node.cpp
)
target_compile_definitions(control_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(control_node
  rclcpp
  rclcpp_components
  geometry_msgs
  moveit_ros_planning_interface
  std_srvs
)
rclcpp_components_register_nodes(control_node
  "pick_place_demo::ControlNode"
)

add_executable(control_node_exe
  src/control_node_main.cpp
)
target_link_libraries(control_node_exe
  control_node
)
set_target_properties(control_node_exe
  PROPERTIES OUTPUT_NAME control_node
)

# Install targets
install(TARGETS
  vision_node
  control_node
  vision_node_exe
  control_node_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install include headers
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_clang_format()

  ament_add_gtest(test_control_node
    test/test_control_node.cpp
  )
  target_link_libraries(test_control_node control_node)

  ament_add_gtest(test_control_node_states
    test/test_control_node_states.cpp
  )
  target_link_libraries(test_control_node_states control_node)

  ament_add_gtest(test_vision_node
    test/test_vision_node.cpp
  )
  target_link_libraries(test_vision_node vision_node)
endif()

ament_package()
