cmake_minimum_required(VERSION 3.0.2)
project(calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  scanAvg_node
    src/averager.cpp)
target_link_libraries(scanAvg_node ${catkin_LIBRARIES})

install(PROGRAMS
  src/reflector_coordinate.py
  src/csv_data.py
  src/binseg.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
