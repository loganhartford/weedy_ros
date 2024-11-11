cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Specify the message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Keypoint.msg"
  "msg/KeypointSet.msg"
  "msg/Inference.msg"
  "msg/Points.msg"
  "msg/CartesianCmd.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Ensure dependencies are fully exported
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rclcpp std_msgs geometry_msgs)
ament_package()
