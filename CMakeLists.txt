cmake_minimum_required(VERSION 3.8)
project(safety_aware_reasoning_ros2_deploy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED) # TODO?
find_package(color_names REQUIRED)
find_package(color_blob_centroid REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dex_ivr_interfaces REQUIRED)
find_package(sar_ros2_interfaces REQUIRED)

# set dependencies
set(dependencies
  rclpy
  tf2_ros
  color_names
  color_blob_centroid
  std_msgs
  vision_msgs
  dex_ivr_interfaces
  sar_ros2_interfaces
)

# find Python package
ament_python_install_package(${PROJECT_NAME})

# install Python executables
install(PROGRAMS
  # TODO
  scripts/hazard_detector_node.py
  scripts/risk_mitigating_action_predictor_node.py
  scripts/safety_reporter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# install additional directories
install(DIRECTORY
  config
  launch
  saved_models
  DESTINATION share/${PROJECT_NAME}
)

# exports dependencies for downstream packages
ament_export_dependencies(${dependencies})

ament_package()
