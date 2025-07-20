cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # Added this line
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(colors_detection_node src/colors_detection_node.cpp)

# Specify dependencies
ament_target_dependencies(colors_detection_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs  # Added this line
  cv_bridge
  image_transport
  OpenCV
)


# Add camera_node executable
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node 
  rclcpp 
  sensor_msgs 
  std_msgs  # Added this line if camera_node also uses std_msgs
  cv_bridge 
  image_transport 
  OpenCV
)

# Install targets
install(TARGETS
  camera_node
  colors_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/color_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()