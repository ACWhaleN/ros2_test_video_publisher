cmake_minimum_required(VERSION 3.8)
project(ros2_test_video_publisher)

# Use C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add -Wall and -Werror to enforce cleaner code
add_definitions(-Wall -Werror)

# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Add library target

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ros2_test_video_publisher.cc
)

# Register node using rclcpp_components
ament_auto_add_executable(ros2_test_video_publisher_node
  src/ros2_test_video_publisher.cc
)
target_include_directories(${PROJECT_NAME}_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS})

# Install launch files
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)

