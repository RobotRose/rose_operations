/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*  Author: Mathijs de Langen
*  Date  : 2014/04/10
*     - File created.
*
* Description:
*  description
* 
***********************************************************************************/
#include "open_gripper/open_gripper_node.hpp"

int main( int argc, char **argv )
{

  string nodename  = "open_gripper";
  // Set up ROS.
  ros::init(argc, argv, nodename);
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  int rate;
  string topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(10));
  private_node_handle_.param("topic", topic, string("/basic_operation/" + nodename));

  // Create a new ScriptInteractionNode object.
  OpenGripper* open_gripper = new OpenGripper(topic, n);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  bool stop = false;

  while (n.ok() && !stop)
  {
    ros::spinOnce();
    r.sleep();
  }

  delete open_gripper;

  return 0;
}