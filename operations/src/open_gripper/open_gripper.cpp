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
#include "open_gripper/open_gripper.hpp"

OpenGripper::OpenGripper( std::string name, ros::NodeHandle n )
	: OperationBaseClass (name, n)
{
	startOperation();
}

OpenGripper::~OpenGripper()
{
	
}

void OpenGripper::CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
	sendResult(true);
}

void OpenGripper::CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
	sendResult(false);
}

void OpenGripper::receiveGoal( const operations::basic_operationGoalConstPtr& goal, SMC* smc )
{
	openAction(); // Overriding choices for items
}

void OpenGripper::openAction()
{
	// ROS_INFO("OpenGripper::openAction()");

	// if ( arm_controller_helper_->getAvailableArms().size() < 1 )
	// 	ROS_ERROR("No arms available");

	// std::string arm = arm_controller_helper_->getAvailableArms().at(0);

	// arm_controller::manipulateGoal goal = arm_controller_helper_->openGripperMessage(arm);

	// smc_->sendGoal<arm_controller::manipulateAction>(goal, arm);
}