cmake_minimum_required(VERSION 3.5)
project(myURDF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)









# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
