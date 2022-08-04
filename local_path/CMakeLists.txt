cmake_minimum_required(VERSION 3.0.2)
project(local_path)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES local_path
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf ackermann_msgs
#  DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} 
src/local_path.cpp
src/pure_pursuit.cpp
src/pid.cpp

include/local_path.h
include/pure_pursuit.h
include/pid.h
include/csv.h
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})
