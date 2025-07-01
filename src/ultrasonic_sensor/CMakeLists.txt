cmake_minimum_required(VERSION 3.8)
project(ultrasonic_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find pigpio
find_library(PIGPIO_IF2_LIBRARY pigpiod_if2)
if(NOT PIGPIO_IF2_LIBRARY)
  message(FATAL_ERROR "pigpio library not found. Install with: sudo apt install libpigpio-dev")
endif()

# Create library for HCSR04
add_library(hcsr04_lib src/libHCSR04.cpp)
target_include_directories(hcsr04_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hcsr04_lib ${PIGPIO_IF2_LIBRARY} pthread)

# Create executable
add_executable(ultrasonic_node src/ultrasonic_node.cpp)
target_include_directories(ultrasonic_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ultrasonic_node hcsr04_lib)

# Add ROS 2 dependencies
ament_target_dependencies(ultrasonic_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install targets
install(TARGETS ultrasonic_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS hcsr04_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install launch files if you create them
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install config files if you create them
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(hcsr04_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
