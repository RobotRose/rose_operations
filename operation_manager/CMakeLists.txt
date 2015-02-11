cmake_minimum_required(VERSION 2.8.3)
project(operation_manager)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	COMPONENTS
	action_planner
	actionlib_msgs
	rose_datamanager_api
	luctor_classes
	operations
	roscomm
	server_multiple_client
)

add_action_files( DIRECTORY action 
	FILES 
    execute.action
)

generate_messages( DEPENDENCIES 
	actionlib_msgs 
	roscomm
)
catkin_package(
	INCLUDE_DIRS 
		include
	#LIBRARIES operation_manager
	CATKIN_DEPENDS 
		rose_datamanager_api 
		luctor_classes 
		roscomm 
		operations 
		action_planner 
		server_multiple_client
	DEPENDS 
		actionlib_msgs
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( operation_manager 	src/operation_manager_node.cpp
									src/operation_manager.cpp
)

add_dependencies( operation_manager ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( operation_manager ${catkin_LIBRARIES} )