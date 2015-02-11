cmake_minimum_required(VERSION 2.8.3)
project(parameter_manager)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	COMPONENTS
	actionlib_msgs
	rose_datamanager_api
	roscomm
	server_multiple_client
)

add_action_files( DIRECTORY action 
	FILES 
    parameter.action
)

generate_messages( DEPENDENCIES 
	actionlib_msgs 
	roscomm
)
catkin_package(
	INCLUDE_DIRS include
	#LIBRARIES parameter_manager
	CATKIN_DEPENDS 
		actionlib_msgs
		rose_datamanager_api 
		roscomm 
		server_multiple_client
	DEPENDS 
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( parameter_manager 	src/parameter_manager_node.cpp
									src/parameter_manager.cpp
)

add_dependencies( parameter_manager ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( parameter_manager ${catkin_LIBRARIES} )