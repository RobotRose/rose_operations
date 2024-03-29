cmake_minimum_required(VERSION 2.8.3)
project(rose_operations)

find_package(catkin REQUIRED
	COMPONENTS
	actionlib 
	actionlib_msgs 
	arm_controller
	geometry_msgs
	luctor_classes	
	operator_messaging
	roscomm
	rose_datamanager_api
	rose_gaze_controller
	rose_transformations
	server_multiple_client
)

set(CMAKE_CXX_FLAGS "-std=c++0x -g")

add_action_files(
	FILES 
    basic_operation.action
)

generate_messages( DEPENDENCIES 
	actionlib_msgs 
	roscomm
)

catkin_package(
	INCLUDE_DIRS 
		include
    LIBRARIES 
    	operation_base_class 
    	manipulation_base_class
	CATKIN_DEPENDS 
		actionlib 
		actionlib_msgs 
		arm_controller
		geometry_msgs
		luctor_classes	
		operator_messaging
		roscomm
		rose_datamanager_api
		rose_gaze_controller
		rose_transformations
		server_multiple_client
	DEPENDS 
		actionlib 
		actionlib_msgs 
		geometry_msgs 
)

include_directories( ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} )

# base class
add_library(operation_base_class ${PROJECT_SOURCE_DIR}/src/operation_base_class/operation_base_class.cpp )
target_link_libraries(operation_base_class ${catkin_LIBRARIES})
add_dependencies(operation_base_class ${PROJECT_NAME}_generate_messages_cpp)

# base class for manipulations
add_library(manipulation_base_class ${PROJECT_SOURCE_DIR}/src/manipulation_base_class/manipulation_base_class.cpp )
target_link_libraries(manipulation_base_class ${catkin_LIBRARIES})
add_dependencies(manipulation_base_class operation_base_class ${PROJECT_NAME}_generate_messages_cpp)

# base class for arm poses
add_library(arm_pose_base_class ${PROJECT_SOURCE_DIR}/src/arm_pose_base_class/arm_pose_base.cpp )
target_link_libraries(arm_pose_base_class operation_base_class ${catkin_LIBRARIES})
add_dependencies(arm_pose_base_class operation_base_class ${PROJECT_NAME}_generate_messages_cpp)

# arm driving position
add_executable( arm_driving_position 	${PROJECT_SOURCE_DIR}/src/arm_driving_position/arm_driving_position_node.cpp
										${PROJECT_SOURCE_DIR}/src/arm_driving_position/arm_driving_position.cpp
)
target_link_libraries( arm_driving_position arm_pose_base_class ${catkin_LIBRARIES})
add_dependencies( arm_driving_position arm_pose_base_class ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# arm grabbing position
add_executable( arm_grabbing_position 	${PROJECT_SOURCE_DIR}/src/arm_grabbing_position/arm_grabbing_position_node.cpp
										${PROJECT_SOURCE_DIR}/src/arm_grabbing_position/arm_grabbing_position.cpp
)
target_link_libraries( arm_grabbing_position arm_pose_base_class ${catkin_LIBRARIES})
add_dependencies( arm_grabbing_position arm_pose_base_class ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# handover to rose
add_executable( handover_to_rose 	${PROJECT_SOURCE_DIR}/src/handover_to_rose/handover_to_rose_node.cpp
						${PROJECT_SOURCE_DIR}/src/handover_to_rose/handover_to_rose.cpp
)
target_link_libraries( handover_to_rose arm_pose_base_class ${catkin_LIBRARIES})
add_dependencies( handover_to_rose 	arm_pose_base_class ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# grab
add_executable( grab 	${PROJECT_SOURCE_DIR}/src/grab/grab_node.cpp
						${PROJECT_SOURCE_DIR}/src/grab/grab.cpp
)
add_dependencies( grab 	${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries( grab operation_base_class manipulation_base_class ${catkin_LIBRARIES})

# place
# add_executable( place 	${PROJECT_SOURCE_DIR}/src/place/place_node.cpp
# 						${PROJECT_SOURCE_DIR}/src/place/place.cpp
# )
# add_dependencies( place 	${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries( place operation_base_class manipulation_base_class ${catkin_LIBRARIES})

# give
# add_executable( give 	${PROJECT_SOURCE_DIR}/src/give/give_node.cpp
# 						${PROJECT_SOURCE_DIR}/src/give/give.cpp
# )
# add_dependencies( give 	${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries( give operation_base_class ${catkin_LIBRARIES})

# move_to
add_executable( move_to 	${PROJECT_SOURCE_DIR}/src/move_to/move_to_node.cpp
							${PROJECT_SOURCE_DIR}/src/move_to/move_to.cpp
	)
target_link_libraries( move_to operation_base_class ${catkin_LIBRARIES} )
add_dependencies( move_to 	${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# say
add_executable( say 	${PROJECT_SOURCE_DIR}/src/say/say_node.cpp
							${PROJECT_SOURCE_DIR}/src/say/say.cpp
	)
target_link_libraries( say operation_base_class ${catkin_LIBRARIES} )
add_dependencies( say 	${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# position_determinator
add_executable( position_determinator 	${PROJECT_SOURCE_DIR}/src/position_determinator/position_determinator_node.cpp
							${PROJECT_SOURCE_DIR}/src/position_determinator/position_determinator.cpp
	)
target_link_libraries( position_determinator operation_base_class ${catkin_LIBRARIES} )
add_dependencies( position_determinator 	${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
