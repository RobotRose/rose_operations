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
	: OperationBaseClass (name, n)
{
	startOperation();
}

HandoverToRose::~HandoverToRose()
{
	
}

void HandoverToRose::CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
	sendResult(true);
}

void HandoverToRose::CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
	sendResult(false);
}

void HandoverToRose::CB_goalReceived(const rose_operations::basic_operationGoalConstPtr& goal, SMC* smc )
{
    bool success = handoverToRoseAction();
    if(success)
    {
        sendResult(true, "Open or close gripper?");
    }
    else
    {
        sendResult(false, "Could not move arms");
    }
}

/*********
  MAIN FUNCTION OF THIS ACTIONS
*********/
// bool HandoverToRose::handoverToRoseAction()
// {
// 	ROS_INFO("HandoverToRose::handover_to_roseAction");

// 	// ArmController::Arms arm = pickArm();

// 	// PoseStamped handover_pose;
// 	// handover_pose.header.frame_id = "arms";
// 	// handover_pose.pose.position.x = 0.0;
// 	// handover_pose.pose.position.y = -0.1;
// 	// handover_pose.pose.position.z = 0.695;

//  //    handover_pose.pose.orientation = rose_conversions::RPYToQuaterion(0,0,0);

// 	// // Execute arm action
//  //    arm_controller::manipulateGoal goal = arm_controller_helper_->moveToPoseMessage( arm, handover_pose, true );
// 	// if ( not armAction ( arm, goal) )		
// 	// 	return false;

//  //    bool opened = true; // = openGripper(arm);

//  //    operator_gui_->action("Open or close gripper?");

//  //    return opened;
//     return false;
// }

// // ArmController::Arms HandoverToRose::pickArm()
// // {
// 	// ArmController::Arms chosen_arm = ArmController::Arms::LEFT;
	
// 	// //! @todo LvB: Check which arm is not holding any object and pick that one
	
// 	// if ( chosen_arm == ArmController::Arms::NONE )
// 	// 	ROS_DEBUG_NAMED(ROS_NAME, "No arm in reach");
// 	// else
// 	// 	ROS_DEBUG_NAMED(ROS_NAME, "Picked %s", arm_controller_helper_->armToString(chosen_arm).c_str());

//  //    return chosen_arm;
// // }

// bool HandoverToRose::armAction( const ArmController::Arms arm, const arm_controller::manipulateGoal goal )
// {
// 	// ROS_DEBUG_NAMED(ROS_NAME, "Sending goal action %d", goal.required_action);

// 	// smc_->sendGoal<arm_controller::manipulateAction>(goal, arm_controller_helper_->getClientFor(arm));
// 	// if (not smc_->waitForResult(arm_controller_helper_->getClientFor(arm), ros::Duration(120.0)))
// 	// 	return false;

// 	// arm_controller::manipulateResultConstPtr result;

// 	// try // result can be NULL
// 	// {
// 	// 	result 				= smc_->getResultLastestClient<arm_controller::manipulateAction>();
// 	// 	result_.return_code = result->return_code;
// 	// }
// 	// catch(...) //! @todo MdL: Add correct catch.
// 	// {
// 	// 	result_.return_code = ARM_COMMUNICATION_ERROR;
// 	// 	sendResult(false);
// 	// 	return false;
// 	// }

// 	// if ( result_.return_code != ACTION_RESULT::SUCCESS )
// 	// {
// 	// 	sendResult(false);
// 	// 	return false;
// 	// }
// 	// else
// 		return true;
// }

// bool HandoverToRose::openGripper( const ArmController::Arms arm )
// {
//     ROS_DEBUG_NAMED(ROS_NAME, "Opening gripper");
//     // operator_gui_->message("Opening gripper");

//     // arm_controller::manipulateGoal goal = arm_controller_helper_->openGripperMessage( arm );
//     // if ( not armAction ( arm, goal) )
//     //     return false;

//     // return true;
// }
