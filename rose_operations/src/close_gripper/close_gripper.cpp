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
#include "close_gripper/close_gripper.hpp"

CloseGripper::CloseGripper( std::string name, ros::NodeHandle n )
	: OperationBaseClass (name, n)
{
	startOperation();
}

CloseGripper::~CloseGripper()
{
	
}

void CloseGripper::CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
	sendResult(true);
}

void CloseGripper::CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
	sendResult(false);
}

void CloseGripper::receiveGoal( const operations::basic_operationGoalConstPtr& goal, SMC* smc )
{
	closeAction(); // Overriding choices for items
}

void CloseGripper::closeAction()
{
	ROS_INFO("CloseGripper::closeAction()");

	// if ( arm_controller_helper_->getAvailableArms().size() < 1 )
	// 	ROS_ERROR("No arms available");

	// std::string arm = arm_controller_helper_->getAvailableArms().at(0);

	// arm_controller::manipulateGoal goal = arm_controller_helper_->closeGripperMessage(arm);

	// smc_->sendGoal<arm_controller::manipulateAction>(goal, arm);
}