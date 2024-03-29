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
#include "arm_grabbing_position/arm_grabbing_position.hpp"

ArmGrabbingPosition::ArmGrabbingPosition( std::string name, ros::NodeHandle n )
	: ArmPoseBaseClass (name, n)
{
    //! @todo MdL [IMPR]: Remove hard-coded arm name.
    //! @todo MdL [CONF]: Configurable pose.
    std::string                arm_name; 
    geometry_msgs::PoseStamped goal_pose;

    arm_name                    = "mico";
    goal_pose.header.stamp      = ros::Time::now();
    goal_pose.header.frame_id   = arm_name;
    goal_pose.pose.position.x   = 0.0;
    goal_pose.pose.position.y   = 0.0;
    goal_pose.pose.position.z   = 0.0;

    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;
    goal_pose.pose.orientation = rose_conversions::RPYToQuaterion(roll, pitch, yaw);

    setCartesianGoal(arm_name, goal_pose);
}

ArmGrabbingPosition::~ArmGrabbingPosition()
{
	
}
