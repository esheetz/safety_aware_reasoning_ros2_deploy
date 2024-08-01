cmake_minimum_required(VERSION 3.8)
project(safety_aware_reasoning_ros2_deploy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

# set dependencies
set(dependencies
  rclpy
  tf2_ros
  std_msgs
)

# find Python package
ament_python_install_package(${PROJECT_NAME})

# install Python executables
install(PROGRAMS
  # TODO
  #scripts/<script-name>.py
  DESTINATION lib/${PROJECT_NAME}
)

# install additional directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# exports dependencies for downstream packages
ament_export_dependencies(${dependencies})

ament_package()
