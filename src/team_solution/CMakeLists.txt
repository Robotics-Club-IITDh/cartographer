cmake_minimum_required(VERSION 3.8)
project(team_solution)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts/nodes
install(PROGRAMS
  team_solution/solution_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
