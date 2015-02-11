/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/02/14
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include "parameter_manager_node.hpp"

int main( int argc, char **argv )
{

// Set up ROS.
    ros::init(argc, argv, "parameter_manager");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    int rate;
    std::string topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("rate", rate, int(10));
    private_node_handle_.param("topic", topic, std::string("parameter_manager"));

    // Create a new ScriptInteractionNode object.
    ParameterManager* parameter_manager = new ParameterManager("parameter_manager", n);

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);

    // Main loop.
    bool stop = false;

    while (n.ok() && !stop)
    {
        ros::spinOnce();
        r.sleep();
    }

    delete parameter_manager;

    return 0;
}