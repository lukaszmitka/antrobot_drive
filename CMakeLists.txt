cmake_minimum_required(VERSION 2.8.3)
project(antrobot_drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES antrobot_drive
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(antrobot_drive
#   src/${PROJECT_NAME}/antrobot_drive.cpp
# )


add_executable(antrobot_drive_node src/antrobot_drive.cpp src/rs232.c)

target_link_libraries(antrobot_drive_node
  ${catkin_LIBRARIES}
)
