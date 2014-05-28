cmake_minimum_required(VERSION 2.8.3)
project(hog_haar_person_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(
	hog_haar_person_detection 
	src/hog_haar_person_detection.cpp
)
target_link_libraries(
	hog_haar_person_detection 
	${catkin_LIBRARIES}
	${OpenCV_INCLUDE_DIRS}
)
