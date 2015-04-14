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
#include "arm_pose_base_class/arm_pose_base.hpp"

ArmPoseBaseClass::ArmPoseBaseClass( std::string name, ros::NodeHandle n )
    : OperationBaseClass (name, n)
    , cartesian_goal_set_ (false)
{
    smc_->addClient<rose_arm_controller_msgs::set_positionAction>("arm_controller/position");
    smc_->addClient<rose_arm_controller_msgs::set_gripper_widthAction>("arm_controller/gripper_width");

    startOperation();
}

ArmPoseBaseClass::~ArmPoseBaseClass()
{
}

void ArmPoseBaseClass::setCartesianGoal( const std::string& arm_name, const geometry_msgs::PoseStamped& goal )
{
    cartesian_goal_[arm_name]   = goal;
    cartesian_goal_set_         = true;
}

bool ArmPoseBaseClass::closeGripper( const std::string& arm_name )
{
    if ( not cartesian_goal_set_ )
        return false;

    return false;
}

bool ArmPoseBaseClass::sendCartesianGoal( const std::string& arm_name, const geometry_msgs::PoseStamped& goal_pose )
{
    ROS_INFO("set cartesian position to arm %s", arm_name.c_str());
    if ( not cartesian_goal_set_ )
    {
        ROS_ERROR("This arm pose operation does not have a cartesian goal set");
        return false;
    }

    rose_arm_controller_msgs::set_positionGoal goal;

    goal.arm            = arm_name;
    goal.required_pose  = goal_pose;

    if ( not smc_->sendGoal<rose_arm_controller_msgs::set_positionAction>(goal, "arm_controller/position"))
    {
        ROS_ERROR("Could not send goal to the arm controller");
        return false;
    }

    if ( not smc_->waitForResult(ros::Duration(20.0)) ) // Wait 20s
        return false;
    else 
        return true;
}

void ArmPoseBaseClass::moveArms()
{
    bool result = true;
    ROS_INFO("Move arms");
    for ( const auto& goal : cartesian_goal_ )
        result &= sendCartesianGoal(goal.first, goal.second);

    sendResult(result);
}

void ArmPoseBaseClass::CB_goalReceived( const rose_operations::basic_operationGoalConstPtr& goal, SMC* smc )
{
    moveArms();
}