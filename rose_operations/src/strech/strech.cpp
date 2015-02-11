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
#include "strech/strech.hpp"

Strech::Strech( std::string name, ros::NodeHandle n )
	: OperationBaseClass (name, n)
{
	startOperation();
}

Strech::~Strech()
{
	
}

void Strech::CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
    sendResult(true);
}

void Strech::CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
    sendResult(false);
}

void Strech::CB_goalReceived( const rose_operations::basic_operationGoalConstPtr& goal, SMC* smc )
{
	strechArms();
}

void Strech::strechArms()
{
    operator_gui_->message("Streching both arms");
	ROS_INFO_NAMED(ROS_NAME, "Strech::strechArm");
    
    ArmController::Arms arm = ArmController::Arms::BOTH;

    std::vector<double> arm_angles;
    for ( int i = 0 ; i < 16 ; i++ )
        arm_angles.push_back(0.0);

    // right arm
    arm_angles.at(0) = -M_PI/2;//0.0;
    arm_angles.at(6) = -M_PI/2;//0.0;
    arm_angles.at(7) = 0.01; // Gripper width

    // left_arm
    arm_angles.at(15) = 0.01; // Gripper width

    arm_controller::manipulateGoal goal = arm_controller_helper_->setJointPositionsMessage( ArmController::Arms::BOTH, arm_angles );

    smc_->sendGoal<arm_controller::manipulateAction>(goal, arm_controller_helper_->getClientFor(arm));
}
