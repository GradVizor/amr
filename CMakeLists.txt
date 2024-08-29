cmake_minimum_required(VERSION 3.5)
project(amr)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add the executable

# Link against rclcpp

# Install the executable
install(TARGETS
  DESTINATION lib/${PROJECT_NAME})

ament_package()
