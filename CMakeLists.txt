cmake_minimum_required(VERSION 2.8.3)
project(acoustic_link)

## Find catkin and any catkin packages
find_package(catkin REQUIRED 
	roscpp 
	rospy 
	std_msgs 
	std_srvs
	control
	safety
	evologics_ros
	)

catkin_package(CATKIN_DEPENDS 
	message_runtime
	rospy
  	roscpp
  	std_msgs
  	std_srvs
	control
	safety
	evologics_ros
  	)

add_message_files(
  FILES
  TopicInfo.msg
)


## Build node
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(acoustic_link_node src/acoustic_link_node.cpp)
target_link_libraries(acoustic_link_node ${catkin_LIBRARIES})


