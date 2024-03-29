cmake_minimum_required(VERSION 2.8.3)
project(rose_action_planner)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") # TODO OH: -g is DEBUG FLAG

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	COMPONENTS
	arm_controller
	rose_datamanager_api
	geometry_msgs
	std_msgs
	rose_pose_explorer
	roscomm
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  recover.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  roscomm
)

catkin_package(
 	INCLUDE_DIRS 
 		include
	LIBRARIES 
	CATKIN_DEPENDS 
		std_msgs 
		arm_controller 
		rose_pose_explorer 
		geometry_msgs
	DEPENDS 
		system_lib
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose_action_planner 	
				src/action_planner_node.cpp
				src/action_planner.cpp
)

add_dependencies( rose_action_planner ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose_action_planner ${catkin_LIBRARIES} )