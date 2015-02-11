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
#include "arm_driving_position/arm_driving_position.hpp"

ArmDrivingPosition::ArmDrivingPosition( std::string name, ros::NodeHandle n )
    : OperationBaseClass (name, n)
{
    startOperation();
}

ArmDrivingPosition::~ArmDrivingPosition()
{
}

void ArmDrivingPosition::CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
}

void ArmDrivingPosition::CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
}

void ArmDrivingPosition::CB_goalReceived( const operations::basic_operationGoalConstPtr& goal, SMC* smc )
{
    moveArmToDrivingPosition();
}

bool ArmDrivingPosition::armAction( const ArmController::Arms& arm, const arm_controller::manipulateGoal& goal )
{
    smc_->sendGoal<arm_controller::manipulateAction>(goal, arm_controller_helper_->getClientFor(arm));
    if (not smc_->waitForResult(ros::Duration(40.0)))
        return false;

    arm_controller::manipulateResultConstPtr result;

    try // result can be NULL
    {
        result              = smc_->getResultLastestClient<arm_controller::manipulateAction>();
        result_.return_code = result->return_code;
    }
    catch(...) //! @todo MdL: Add correct catch.
    {
        result_.return_code = ARM_COMMUNICATION_ERROR;
        return false;
    }

    if ( result_.return_code != ACTION_RESULT::SUCCESS )
        return false;
    else
        return true;
}

bool ArmDrivingPosition::closeGripper( const ArmController::Arms& arm )
{
    arm_controller::manipulateGoal goal = arm_controller_helper_->closeGripperMessage( arm );
    return armAction ( arm, goal);
}

bool ArmDrivingPosition::drivingPosition( const ArmController::Arms& arm )
{
    double          roll    = M_PI/2; // rotations of the gripper
    double          pitch   = 0.0;//M_PI/6;
    double          yaw     = 0.0;

    arm_controller::manipulateGoal goal;
    PoseStamped pose;

    //! @todo MdL: poses in base link coordinates!
    pose.header.frame_id  = "arms";
    pose.header.stamp     = ros::Time::now();

    //! @todo MdL: Fix nicely.
    if (arm == ArmController::Arms::LEFT)
    {
        pose.pose.position.x   = -0.23;
        pose.pose.position.y   = -0.32;
        pose.pose.position.z   = 0.19;
        pose.pose.orientation.w = 1.0;

        goal = arm_controller_helper_->moveToPoseMessage( arm, pose, true );

        goal.required_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw); // in left_arm frame
    }

    if (arm == ArmController::Arms::RIGHT)
    {
        pose.pose.position.x  = 0.23;
        pose.pose.position.y  = -0.32;
        pose.pose.position.z  = 0.20;
        pose.pose.orientation.w = 1.0;
        goal = arm_controller_helper_->moveToPoseMessage( arm, pose, true );

        pitch = M_PI;
        goal.required_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    }

    if ( not armAction(arm, goal) )
    {
        ROS_ERROR("End pose (%s) not reached", arm_controller_helper_->armToString(arm).c_str());
        return false;
    }

    return true;
}

void ArmDrivingPosition::moveArmToDrivingPosition()
{
    operator_gui_->message("Arm inklappen (voor rijden)");

    std::vector<ArmController::Arms> arms = arm_controller_helper_->getArms();
    // Close both grippers
     for ( const auto& arm : arms )
        if ( not closeGripper(arm) )
        {
            sendResult(false);
            return;
        }

    for ( const auto& arm : arms )
        if ( not drivingPosition(arm))
        {
            sendResult(false);
            return;
        }

    // To make sure there are on their place
    // Sometimes, arm X moves as arm Y moves to a pose
    // for ( const auto& arm : arms )
        // drivingPosition(arm);

    sendResult(true);
}
