cmake_minimum_required(VERSION 3.8)
project(scara)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rviz2 REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_publisher_gui REQUIRED)

# Install Python nodes
install(PROGRAMS
  src/pendulum_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz configuration
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()