/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/04/10
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include "handover_to_rose/handover_to_rose.hpp"

HandoverToRose::HandoverToRose( std::string name, ros::NodeHandle n )
	: ArmPoseBaseClass (name, n)
{
    //! @todo MdL [IMPR]: Remove hard-coded arm name.
    //! @todo MdL [CONF]: Configurable pose.
    std::string                arm_name; 
    geometry_msgs::PoseStamped goal_pose;

    arm_name                    = "mico";
    goal_pose.header.stamp      = ros::Time::now();
    goal_pose.header.frame_id   = arm_name;
    goal_pose.pose.position.x   = -0.2;
    goal_pose.pose.position.y   = -0.4;
    goal_pose.pose.position.z   = 0.3;

    double roll  = -M_PI/2;
    double pitch = 0.0; // correct
    double yaw   = 0.0; // correct

    goal_pose.pose.orientation = rose_conversions::RPYToQuaterion(roll, pitch, yaw);

    setCartesianGoal(arm_name, goal_pose);
}

HandoverToRose::~HandoverToRose()
{
	
}
