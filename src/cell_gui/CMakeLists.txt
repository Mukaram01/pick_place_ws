cmake_minimum_required(VERSION 3.8)
project(cell_gui)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Instruct CMake to create code from Qt designer ui files
set(CMAKE_AUTOUIC ON)
# Instruct CMake to run rcc automatically when needed
set(CMAKE_AUTORCC ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

# Include directories
include_directories(include)

# Add UI files
set(UI_FILES
  ui/main_window.ui
)

# Process UI files
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

# Add executable
set(RESOURCE_FILES
  resources/icons.qrc
)

add_executable(cell_gui_node
  src/main.cpp
  src/main_window.cpp
  ${UI_HEADERS}
  ${RESOURCE_FILES}
)

# Add dependencies
ament_target_dependencies(cell_gui_node
  rclcpp
  std_msgs
)

# Link Qt libraries
target_link_libraries(cell_gui_node
  Qt5::Widgets
  Qt5::Core
)

# Install the executable
install(TARGETS
  cell_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY ui/
  DESTINATION share/${PROJECT_NAME}/ui
)

# Install resources
install(
  DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
  PATTERN ".svn" EXCLUDE
)

ament_package()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_clang_format()
endif()
