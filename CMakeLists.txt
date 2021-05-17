cmake_minimum_required(VERSION 3.5)
project(fog_gazebo_resources)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY
  worlds
  )

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
