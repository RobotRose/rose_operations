/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
* Author: Loy van Beek
* Date  : 2014/09/08
*     - File created.
* 
***********************************************************************************/
#include "manipulation_base_class/manipulation_base_class.hpp"

ManipulationBaseClass::ManipulationBaseClass( std::string name, ros::NodeHandle n )
	: OperationBaseClass (name, n)
{
	startOperation();

	toggle_visual_correction_service_ = n_.serviceClient<arm_controller::toggle_visual_correction>("/arms/toggle_visual_correction");
	reset_visual_correction_service_  = n_.serviceClient<arm_controller::reset_visual_correction>("/arms/reset_visual_correction");

	gaze_client_ = new GazeClient("gaze_controller", true); 
}

ManipulationBaseClass::~ManipulationBaseClass()
{
	
}

bool ManipulationBaseClass::visualServoingAction ( const arm_controller::move_to_tfGoal goal )
{
	smc_->sendGoal<arm_controller::move_to_tfAction>(goal, "arm_visual_servoing");
	if (not smc_->waitForResult("arm_visual_servoing", ros::Duration(120.0)))
		return false;

	arm_controller::move_to_tfResultConstPtr result;

	try // result can be NULL
	{
		//! @todo MdL: Still generates segfault.
		result 				= smc_->getResultLastestClient<arm_controller::move_to_tfAction>();
		result_.return_code = result->return_code;
	}
	catch(...) //! @todo MdL: Add correct catch.
	{
		result_.return_code = ARM_COMMUNICATION_ERROR;
		// sendResultAndCleanup(false, item);
		return false;
	}

	if ( result_.return_code != ACTION_RESULT::SUCCESS )
	{
		// sendResultAndCleanup(false, item);
		return false;
	}
	else
		return true;
}

bool ManipulationBaseClass::armAction( const ArmController::Arms arm, const arm_controller::manipulateGoal goal )
{
    ROS_INFO_NAMED(ROS_NAME, "Sending goal action of type %d", goal.required_action);

	smc_->sendGoal<arm_controller::manipulateAction>(goal, arm_controller_helper_->getClientFor(arm));
	if (not smc_->waitForResult(arm_controller_helper_->getClientFor(arm), ros::Duration(0.0)))
		return false;

	arm_controller::manipulateResultConstPtr result;

	try // result can be NULL
	{
		result 				= smc_->getResultLastestClient<arm_controller::manipulateAction>();
        if(!result)
            ROS_ERROR_NAMED(ROS_NAME, "smc_->getResultLastestClient<arm_controller::manipulateAction>() has no result");

		result_.return_code = result->return_code;
	}
	catch(...) //! @todo MdL: Add correct catch.
	{
        ROS_DEBUG_NAMED(ROS_NAME, "Some exception occurred in ManipulationBaseClass::armAction after smc_->getResultLastestClient<arm_controller::manipulateAction>();");
		result_.return_code = ARM_COMMUNICATION_ERROR;
		return false;
	}

	if ( result_.return_code != ACTION_RESULT::SUCCESS )
		return false;
	else
		return true;
}

ArmController::Arms ManipulationBaseClass::pickArm( const vector<PoseStamped> poses )
{
	ArmController::Arms chosen_arm = ArmController::Arms::NONE;
	int prev_reachable_points = 0;
    int reachable_points = -1;

	ROS_DEBUG_NAMED(ROS_NAME, "ManipulationBaseClass::pickArm");
	for ( auto& arm : arm_controller_helper_->getArms() )
	{
		reachable_points = getNrOfReachablePoints(arm, poses);
        ROS_DEBUG_NAMED(ROS_NAME, "%s can reach %i of %i poses", arm_controller_helper_->armToString(arm).c_str(), reachable_points, (int)poses.size());
		if ( reachable_points > prev_reachable_points)
		{
			chosen_arm = arm;
			prev_reachable_points = reachable_points;
		}
		
	}
	
	if ( chosen_arm == ArmController::Arms::NONE )
		ROS_DEBUG_NAMED(ROS_NAME, "No arm in reach");
	else
        ROS_DEBUG_NAMED(ROS_NAME, "Picked %s. Can reach %i of %i poses", arm_controller_helper_->armToString(chosen_arm).c_str(), prev_reachable_points, (int)poses.size());

	return chosen_arm;
}

bool ManipulationBaseClass::reachable( const ArmController::Arms arm, const vector<PoseStamped> poses )
{
	bool result = true;
	for ( auto& pose : poses )
		result = result && arm_controller_helper_->reachable(arm, pose);

	return result;
}

int ManipulationBaseClass::getNrOfReachablePoints( const ArmController::Arms arm, const vector<PoseStamped> poses )
{
	int nr_reachable_points_found = 0;
	for ( auto& pose : poses )
    {
		if ( arm_controller_helper_->reachable(arm, pose) )
        {
			nr_reachable_points_found++;
        }
    }
		
	return nr_reachable_points_found;
}

void ManipulationBaseClass::createTransform( const std::string name, const PoseStamped pose_stamped )
{
	ROS_DEBUG_NAMED(ROS_NAME, "createTransform");

	// Map does not contain this transform
	if ( transforms_.find(name) == transforms_.end())
	{
		transforms_mutex_.lock();
		TFHelper* transform = new TFHelper(name, n_, pose_stamped.header.frame_id, name);
		transform->setTransform(pose_stamped);
		transforms_.insert(std::pair<std::string, TFHelper*>(name, transform));
		transforms_mutex_.unlock();
	}
	else
		updateTransform(name, pose_stamped);
}

void ManipulationBaseClass::updateTransform( const std::string name, const PoseStamped pose_stamped ) 
{
	ROS_DEBUG_NAMED(ROS_NAME, "updateTransform");
	//! @todo MdL: Check if transform is already there.
    if ( transforms_.find(name) == transforms_.end())
    {
        createTransform(name, pose_stamped);
    }
    else
    {
        transforms_mutex_.lock();
        transforms_.find(name)->second->setTransform(pose_stamped);
        transforms_mutex_.unlock();
    }

}

void ManipulationBaseClass::removeTransform( const std::string name )
{
	ROS_DEBUG_NAMED(ROS_NAME, "removeTransform");
	transforms_mutex_.lock();

	//! @todo MdL: Check if transform is there.
    if ( transforms_.find(name) != transforms_.end())
    {
        delete transforms_.find(name)->second;
        transforms_.erase(name);
    }
    else
    {
        ROS_WARN_NAMED(ROS_NAME, "Could not remove transform %s because it is not in our map", name.c_str());
    }

	transforms_mutex_.unlock();
}

void ManipulationBaseClass::broadcastTransforms()
{
	// ROS_DEBUG_NAMED(ROS_NAME, "broadcastTransforms");
	transforms_mutex_.lock();

	if ( (transforms_.size() > 0 ) )
		for ( auto& tf : transforms_)
			tf.second->Broadcast();

	transforms_mutex_.unlock();
}

bool ManipulationBaseClass::openGripper( const ArmController::Arms arm )
{
	ROS_DEBUG_NAMED(ROS_NAME, "Opening gripper");
	operator_gui_->message("Opening gripper");

    arm_controller::manipulateGoal goal = arm_controller_helper_->openGripperMessage( arm );
	if ( not armAction ( arm, goal) )
		return false;

	return true;
}

bool ManipulationBaseClass::closeGripper( const ArmController::Arms arm )
{
	ROS_DEBUG_NAMED(ROS_NAME, "Closing gripper");
	operator_gui_->message("Closing gripper");

	arm_controller::manipulateGoal goal = arm_controller_helper_->closeGripperMessage( arm );
	if ( not armAction ( arm, goal) )
		return false;

	return true;
}


//! @todo MdL: Test basic moving of end effector.
//! Replaces post_manipulation_base_class
bool ManipulationBaseClass::moveGripperUp( const ArmController::Arms arm, const double z, bool constrained)
{
	ROS_INFO_NAMED(ROS_NAME, "Move gripper up");

	// Initialize pose stamped in arms frame
	PoseStamped move_up;
	move_up.header.frame_id 	= "base_link";
	move_up.header.stamp 		= ros::Time::now();
	move_up.pose.position.z 	= z;
	move_up.pose.orientation.w 	= 1.0;

	ROS_INFO_NAMED(ROS_NAME, "Generating message");
    arm_controller::manipulateGoal goal = arm_controller_helper_->moveToRelativePoseMessage( arm, move_up, constrained );
	if ( not armAction ( arm, goal) )
		return false;

	return true;
}

void ManipulationBaseClass::removeBoundingBoxOfItemFromDatabase( Item item )
{
	BoundingBox empty_bb;
	item.set_bounding_box(empty_bb);

	datamanager_->store<Item>(item);
}

//! @todo MdL: Make library (arm uses exactly the same).
const bool ManipulationBaseClass::lookAt( const std::string frame_id, const bool keep_tracking)
{
	operator_gui_->message("Looking towards item");
    rose20_gaze_controller::LookAtGoal goal;

    goal.target_point.header.frame_id   = frame_id;
    goal.target_point.point.x           = 0;
    goal.target_point.point.y           = 0;
    goal.target_point.point.z           = 0.05;
    goal.keep_tracking                  = keep_tracking;

    ROS_DEBUG_NAMED(ROS_NAME, "Going to look to %s...", frame_id.c_str());
    gaze_client_->sendGoal(goal);

    gaze_client_->waitForResult(ros::Duration(4.0));
    if ( gaze_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED )
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Succeeded");
        return true;
    }

    ROS_DEBUG_NAMED(ROS_NAME, "Failed");
    return false;
}

