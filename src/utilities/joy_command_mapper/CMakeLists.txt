cmake_minimum_required(VERSION 3.5)
project(joy_command_mapper)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotCommand.msg"
  DEPENDENCIES std_msgs
)

# Build executable
add_executable(joy_command_mapper_node src/joy_command_mapper.cpp)

ament_target_dependencies(joy_command_mapper_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Link against the message library
rosidl_target_interfaces(joy_command_mapper_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install executables
install(TARGETS
  joy_command_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()