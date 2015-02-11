cmake_minimum_required(VERSION 2.8.3)
project(rose_script_manager)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED 
	COMPONENTS 
    actionlib 
    actionlib_msgs 
    genmsg 
    message_generation 
    roscomm
    rospy 
    roscpp 
    std_msgs
)

add_message_files(DIRECTORY msg
    FILES
    current_action.msg
)

add_action_files(DIRECTORY action 
	FILES 
    execute_script.action
)

generate_messages(DEPENDENCIES actionlib_msgs roscomm)

catkin_package(
    INCLUDE_DIRS 
    LIBRARIES 
    CATKIN_DEPENDS 
        actionlib
        actionlib_msgs
        std_msgs
    DEPENDS 
        message_runtime
)
