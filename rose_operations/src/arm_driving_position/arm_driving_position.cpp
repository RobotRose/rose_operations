/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Mathijs de Langen
*   Date  : 2014/04/10
*       - File created.
*
* Description:
*   description
* 
***********************************************************************************/
#include "arm_driving_position/arm_driving_position.hpp"

ArmDrivingPosition::ArmDrivingPosition( std::string name, ros::NodeHandle n )
    : ArmPoseBaseClass (name, n)
{
    //! @todo MdL [IMPR]: Remove hard-coded arm name.
    //! @todo MdL [CONF]: Configurable pose.
    std::string                arm_name; 
    geometry_msgs::PoseStamped goal_pose;

    arm_name                    = "mico";
    goal_pose.header.stamp      = ros::Time::now();
    goal_pose.header.frame_id   = arm_name;
    goal_pose.pose.position.x   = -0.224;
    goal_pose.pose.position.y   = -0.179;
    goal_pose.pose.position.z   = 0.281;

    double roll  = -1.443;
    double pitch = -0.110;
    double yaw   = -1.474;

    goal_pose.pose.orientation = rose_conversions::RPYToQuaterion(roll, pitch, yaw);

    setCartesianGoal(arm_name, goal_pose);
}

ArmDrivingPosition::~ArmDrivingPosition()
{

}
