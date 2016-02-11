cmake_minimum_required(VERSION 2.8.3)
project(acoustic_link)

## Find catkin and any catkin packages
find_package(catkin REQUIRED 
	roscpp 
	rospy 
	std_msgs 
	std_srvs
	cola2_control
	evologics_driver
	)

catkin_package(CATKIN_DEPENDS 
	message_runtime
	rospy
  	roscpp
  	std_msgs
  	std_srvs
  	cola2_control
	evologics_driver
  	)


## Build node
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(topic_out_node src/topic_out_node.cpp)
target_link_libraries(topic_out_node ${catkin_LIBRARIES})

#add_executable(service_out_node src/service_out_node.cpp)
#target_link_libraries(service_out_node ${catkin_LIBRARIES})

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})
