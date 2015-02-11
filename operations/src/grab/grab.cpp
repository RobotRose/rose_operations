/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
* Author: Mathijs de Langen
* Date  : 2014/02/14
*     - File created.
*
* Description:
*   Grab class. Grabs item stored in the database. Does basic visual servoing.
* 
***********************************************************************************/
#include "grab/grab.hpp"

Grab::Grab( std::string name, ros::NodeHandle n )
    : ManipulationBaseClass (name, n)
{

}

Grab::~Grab()
{
	
}

void Grab::CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{

}

void Grab::CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
    operator_gui_->warn("Arm gefaald");
}

void Grab::CB_armVisualServoingSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::move_to_tfResultConstPtr& result )
{

}

void Grab::CB_armVisualServoingFail( const actionlib::SimpleClientGoalState& state, const arm_controller::move_to_tfResultConstPtr& result )
{
    // operator_gui_->warn("Grab may fail");
}

void Grab::CB_armVisualServoingFeedback( const arm_controller::move_to_tfFeedbackConstPtr& feedback )
{
	//! @todo MdL: Move table pose(?).
}

void Grab::CB_serverCancel( SMC* smc )
{
    operator_gui_->message("Cancel ontvangen");
}

void Grab::executeItemAction( const std::string item_id, const vector<std::string> parameters )
{
	ROS_INFO("Grab::executeItemAction");
	Item item = datamanager_->get<Item>(item_id);

	if (!item.get_bounding_box().isSet())
	{
		operator_gui_->action("Selecteer item");
		getParameter(item_id, PARAMETER_REQUEST::BOUNDING_BOX, boost::bind(&Grab::continueItemAction, this, _1, _2));
	}
	else
		preGrabAction( item_id, parameters );
}

void Grab::continueItemAction( const std::string item_id, const vector<std::string> parameters )
{
	ROS_INFO("Grab::continueItemAction");
	Item item = datamanager_->get<Item>(item_id);
	if (!item.get_bounding_box().isSet())
	{
		result_.return_code = MISSING_PARAMETER_ERROR;
		sendResult( false );
	}
	else
		preGrabAction ( item_id, parameters );
}

bool Grab::getTablePose( Item item, PoseStamped& pose )
{
	bool 	result 				= getGrabPose(item, pose);
	double 	distance_to_table 	= std::max(MIN_GRIPPER_TABLE_DIST_DOWN, (item.get_bounding_box().getHeigth() / 2));

	ROS_DEBUG_NAMED(ROS_NAME, "Transform table pose");
	    
	result = result && rose_transformations::addXYZInFrame(tf_, "base_link", 0.0, 0.0, - (distance_to_table + TABLE_CORRECTION + TABLE_HEIGHT / 2), pose);

	// pose is in base_link
	// pose.pose.position.x = (TABLE_HEIGHT/2) + (BASE_LINK_LENGTH/2) + 0.15;
	//pose.pose.position.x = (TABLE_HEIGHT/2) + (BASE_LINK_LENGTH/2) + MIN_GRIPPER_TABLE_DIST_FORW;

	return result;
}

bool Grab::getDynamicPreGrabPose( const ArmController::Arms arm, Item& item, PoseStamped& pose )
{
	rose_transformations::transformToFrameNow(tf_, "base_link", pose);

	sensor_msgs::PointCloud reachable_points = arm_controller_helper_->getReachablePointCloud(arm);
	rose_transformations::transformToFrameNow(tf_, "base_link", reachable_points);

	geometry_msgs::Point result;

	double current_distance = 100.0;

	ROS_DEBUG_NAMED(ROS_NAME, "Trying to find point close to (%f, %f, %f).", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

	for ( auto& reachable_point : reachable_points.points )
	{
		// In baselink coordinates
		if (   reachable_point.y > pose.pose.position.y + 0.075 
			|| reachable_point.y < pose.pose.position.y - 0.075
			|| reachable_point.x > pose.pose.position.x 
			|| reachable_point.z < pose.pose.position.z )
		{
			ROS_DEBUG_NAMED(ROS_NAME, "Disregarding (%f, %f, %f). Distance %f", reachable_point.x, reachable_point.y, reachable_point.z, rose_geometry::distanceXYZ(pose.pose.position, rose_conversions::point32ToPoint(reachable_point)));
		}
		else
		{
			double distance = rose_geometry::distanceXYZ(pose.pose.position, rose_conversions::point32ToPoint(reachable_point));
			ROS_DEBUG_NAMED(ROS_NAME, "Looking at (%f, %f, %f). Distance %f", reachable_point.x, reachable_point.y, reachable_point.z, distance);
			if ( distance < current_distance )
			{
				result     	     = rose_conversions::point32ToPoint(reachable_point);
				current_distance = distance;
			}
		}
	}

	ROS_DEBUG_NAMED(ROS_NAME, "Point found (%f, %f, %f).", result.x, result.y, result.z);

	// too far
	if ( false /*current_distance > 0.25*/ )
    {
        ROS_ERROR_NAMED(ROS_NAME, "Grab::getDynamicPreGrabPose: currentDistance %2.3f > 0.10", current_distance);
		return false;
    }
    else
    {
		pose.pose.position = result;
    	ROS_DEBUG_NAMED(ROS_NAME, "Grab::dynamic pre grab at (%f, %f, %f)", result.x, result.y, result.z);
    }

	return true;
}

bool Grab::getPreGrabPose( Item item, PoseStamped& pose )
{
	ROS_DEBUG_NAMED(ROS_NAME, "getPreGrabPose");

	double pre_grab = std::max(MIN_PRE_GRAB_X, (PRE_GRAB_X + (item.get_bounding_box().getDepth() / 2)));

    // ROS_DEBUG_NAMED(ROS_NAME, "getPreGrabPose. pre_grab =       = std::max(MIN_PRE_GRAB_X, (PRE_GRAB_X + (item.get_bounding_box().getDepth() / 2)));");
    // ROS_DEBUG_NAMED(ROS_NAME, "getPreGrabPose. pre_grab = %2.3f = std::max(%2.12f, (%2.8f + (%2.8f / 2)));",
    //                 pre_grab, MIN_PRE_GRAB_X, PRE_GRAB_X, item.get_bounding_box().getDepth());

	if ( (item.get_bounding_box().getHeigth() / 2) < MIN_GRIPPER_TABLE_DIST_DOWN )
    	return getPoseRelativeToGrab( item, pose, - pre_grab, 0.0, TABLE_CORRECTION );
	else
    	return getPoseRelativeToGrab( item, pose, - pre_grab, 0.0, TABLE_CORRECTION );
}

bool Grab::getGrabPose( Item item, PoseStamped& pose )
{
	ROS_DEBUG_NAMED(ROS_NAME, "getGrabPose");
	return getPoseRelativeToGrab( item, pose, 0.02, 0.0, 0.0 ); // Plus x 0.02 HACK
}

bool Grab::getPostGrabPose( Item item, PoseStamped& pose )
{
	ROS_DEBUG_NAMED(ROS_NAME, "getPostPose");
	return getPoseRelativeToGrab( item, pose, 0.0, 0.0, POST_GRAB_Z );
}

bool Grab::getPoseRelativeToGrab( Item item, PoseStamped& pose, const double x, const double y, const double z )
{
	std::string item_id = item.get_id();

	if ( not rose_transformations::getFrameInFrame(tf_, item_id, "base_link", pose))
    {
        ROS_ERROR_NAMED(ROS_NAME, "Could not getFrameInFrame for %s to base_link", item_id.c_str());
		return false;
    }

	if ( not rose_transformations::addXYZInFrame(tf_, "base_link", x, y, z, pose))
    {
        ROS_ERROR_NAMED(ROS_NAME, "Could not addXYZInFrame(/base_link, %2.3f, %2.3f, %2.3f) for frame %s", x, y, z, pose.header.frame_id.c_str());
		return false;
    }

	return true;
}

bool Grab::preGrabStatic( const ArmController::Arms arm, Item item )
{
	if ( not preGrab(arm, item) )
		return false;

	PoseStamped pre_grab;
	if ( not getPreGrabPose(item, pre_grab))
		return false;

	ROS_DEBUG_NAMED(ROS_NAME, "Pre grab at x: %f, y: %f, z: %f in frame %s", pre_grab.pose.position.x, pre_grab.pose.position.y, pre_grab.pose.position.z, pre_grab.header.frame_id.c_str());

	// Execute arm action
    arm_controller::manipulateGoal goal = arm_controller_helper_->moveToPoseMessage( arm, pre_grab, true );
	if ( not armAction ( arm, goal) )		
		return false;

	return true;
}

bool Grab::preGrabDynamic( const ArmController::Arms arm, Item item )
{
	PoseStamped pre_grab;

	if ( not preGrab(arm, item) )
		return false;

	if ( not getPreGrabPose(item, pre_grab))
    {
        ROS_ERROR_NAMED(ROS_NAME, "Grab::preGrabDynamic: could not getPreGrabPose");
		return false;
    }

	if ( not getDynamicPreGrabPose(arm, item, pre_grab))
    {
        ROS_ERROR_NAMED(ROS_NAME, "Grab::preGrabDynamic: could not getDynamicPreGrabPose");
		return false;
    }

	// Execute arm action
	arm_controller::manipulateGoal goal;
	

	ROS_DEBUG_NAMED(ROS_NAME, "Pre grab at x: %f, y: %f, z: %f in frame %s", pre_grab.pose.position.x, pre_grab.pose.position.y, pre_grab.pose.position.z, pre_grab.header.frame_id.c_str());

    goal 		 = arm_controller_helper_->moveToPoseMessage( arm, pre_grab, true );
    if ( not armAction( arm, goal) )
    {
    	return false;
    }

	return true;
}

bool Grab::preGrab( const ArmController::Arms arm, Item item )
{
	ROS_DEBUG_NAMED(ROS_NAME, "pre grab");

	PoseStamped grab;
	if ( not getGrabPose(item, grab))
		return false;

	PoseStamped table_pose;
    if ( not getTablePose(item, table_pose))
    	return false;
    
	std::map<ArmController::Manipulators, geometry_msgs::PoseStamped> obstacles;
	obstacles.insert(std::pair<ArmController::Manipulators, geometry_msgs::PoseStamped>(ArmController::Manipulators::BOX, grab));
	obstacles.insert(std::pair<ArmController::Manipulators, geometry_msgs::PoseStamped>(ArmController::Manipulators::TABLE, table_pose));

	arm_controller::manipulateGoal goal = arm_controller_helper_->setObstaclesMessage( obstacles );
	if ( not armAction ( arm, goal) )		
		return false;

	return true;
}

bool Grab::grab( const ArmController::Arms arm, Item item )
{
	ROS_DEBUG_NAMED(ROS_NAME, "grab");
	// Remove BOX before grab action. Otherwise the arm will collide in the simulator and not move.
	vector<ArmController::Manipulators> obstacles;
	obstacles.push_back(ArmController::Manipulators::BOX);
	arm_controller::manipulateGoal goal = arm_controller_helper_->removeObstaclesMessage(obstacles);
	if ( not armAction ( arm, goal) )
		return false;

	PoseStamped grab;
	if ( not getGrabPose(item, grab))
		return false;

	ROS_DEBUG_NAMED(ROS_NAME, "Grab at x: %f, y: %f, z: %f in frame %s", grab.pose.position.x, grab.pose.position.y, grab.pose.position.z, grab.header.frame_id.c_str());

    goal = arm_controller_helper_->moveToPoseMessage( arm, grab, true );
	if ( not armAction ( arm, goal) )
		return false;

	return true;
}

bool Grab::postGrab( const ArmController::Arms arm, Item item )
{
	ROS_DEBUG_NAMED(ROS_NAME, "post grab");

	PoseStamped post_grab;
	if ( not getPostGrabPose(item, post_grab))
		return false;

	ROS_DEBUG_NAMED(ROS_NAME, "Post grab at x: %f, y: %f, z: %f in frame %s", post_grab.pose.position.x, post_grab.pose.position.y, post_grab.pose.position.z, post_grab.header.frame_id.c_str());
    arm_controller::manipulateGoal goal = arm_controller_helper_->moveToPoseMessage( arm, post_grab, false );
	if ( not armAction ( arm, goal) )
		return false;

	return true;
}

bool Grab::moveTableUp( const ArmController::Arms arm, Item item, const double height )
{
	PoseStamped table_pose;
	if ( not getTablePose(item, table_pose))
		return false;

	if ( not rose_transformations::addXYZInFrame(tf_, "base_link", 0.0, 0.0, height, table_pose) )
	{
		ROS_ERROR("Could not change table position");
		return false;
	}

	std::map<ArmController::Manipulators, geometry_msgs::PoseStamped> table_obstacle_map;
	table_obstacle_map.insert(std::pair<ArmController::Manipulators, geometry_msgs::PoseStamped>(ArmController::Manipulators::TABLE, table_pose));

	arm_controller::manipulateGoal goal = arm_controller_helper_->setObstaclesMessage( table_obstacle_map );
	if ( not armAction ( arm, goal) )	
	{
		ROS_DEBUG_NAMED(ROS_NAME, "Could not set new table position");
		return false;
	}
	
	ROS_DEBUG_NAMED(ROS_NAME, "Table position changed.");

	return true;
}

//! @todo MdL: Test basic 'visual servoing'.
//! Replaces grab
bool Grab::moveTowardsItem( const ArmController::Arms arm, Item item )
{
	// Remove BOX before grab action. Otherwise the arm will collide in the simulator and not move.
	vector<ArmController::Manipulators> obstacles;
	obstacles.push_back(ArmController::Manipulators::BOX);
	arm_controller::manipulateGoal goal = arm_controller_helper_->removeObstaclesMessage(obstacles);	
	if ( not armAction ( arm, goal) )
		return false;

	std::string item_id = item.get_id();

	ROS_DEBUG_NAMED(ROS_NAME, "Move towards grab");
	operator_gui_->message("Beweging richting item");

	if ( DO_VISUAL_SERVOING )
	{
		arm_controller::move_to_tfGoal servoing_goal;

		servoing_goal.arm 			= arm;
		servoing_goal.tf  			= item_id;
		// First move in xy (arm frame)
		servoing_goal.max_xy_error  = 0.018;

		if (not visualServoingAction (servoing_goal) )
			return false;

		return true;
		
	}
	else
	{
		// Old implementation
		PoseStamped error;
		if ( not rose_transformations::getFrameInFrame(tf_, item_id, arm_controller_helper_->armToString(arm)+"_observed_gripper_tip", error, 5) )
		{
			ROS_ERROR("Could not find error");
			operator_gui_->warn("Kon de marker niet vinden");
			return false;
		}
			
		// Get the relative pose message already
		arm_controller::manipulateGoal arm_goal = arm_controller_helper_->moveToRelativePoseMessage( arm, error, true );

		Pose zero;
		ROS_DEBUG_NAMED(ROS_NAME, "Found distance to %s: %f", item_id.c_str(), rose_geometry::distanceXYZ(zero, error.pose));

		

		// Arm has to move downwards, move table down
		if ( arm_goal.required_pose.position.y < 0 )
			moveTableUp( arm, item, arm_goal.required_pose.position.y );

		// First move in the (x,y)-position of the arm. After that the z-coordinate.
		// This make sure the arm move in a straight line towards the object.

		double z_temp = arm_goal.required_pose.position.z;
		double y_temp = arm_goal.required_pose.position.y;

		arm_goal.required_pose.position.y = 0.0;
		arm_goal.required_pose.position.z = 0.0;
		// Move towards new position (1)
	    if ( not armAction ( arm, arm_goal) )
	    {
	    	ROS_ERROR("Could not move to error correction (1)");
			return false;
	    }

	    ROS_DEBUG_NAMED(ROS_NAME, "Moved to first error correction.");

	    arm_goal.required_pose.position.x = 0.0;
	    arm_goal.required_pose.position.y = y_temp;
	    arm_goal.required_pose.position.z = z_temp;

	    // Move towards new position (2)
	    if ( not armAction ( arm, arm_goal) )
	    {
	    	ROS_ERROR("Could not move to error correction (2)");
			return false;
	    }

	    ROS_DEBUG_NAMED(ROS_NAME, "Moved to error correction.");

	    // Arm had to move upwards, move table up
		// if ( arm_goal.required_pose.position.y > 0 )
		// 	moveTableUp( arm, item, arm_goal.required_pose.position.y );
		return true;
	}

	// This statement should never be reached
	return false;
}

bool Grab::setTable( Item item )
{
	PoseStamped table_pose;
    if ( not getTablePose(item, table_pose))
    	return false;
    
	std::map<ArmController::Manipulators, geometry_msgs::PoseStamped> obstacles;
	obstacles.insert(std::pair<ArmController::Manipulators, geometry_msgs::PoseStamped>(ArmController::Manipulators::TABLE, table_pose));

	arm_controller::manipulateGoal goal = arm_controller_helper_->setObstaclesMessage( obstacles );
	ArmController::Arms arm = ArmController::Arms::NONE; // Do not care about arm
	if ( not armAction ( arm, goal) )		
		return false;

	return true;
}

void Grab::preGrabAction( const std::string item_id, const vector<std::string> parameters )
{
	operator_gui_->message("Denken...");

	// Get item and bounding box (do this first, because transform creation/syncing takes some time)
	Item item 	   = datamanager_->get<Item>(item_id);
	createTransform(item_id, item.get_bounding_box().getPoseStamped());
	
 	// Clear other obstacles and set the BOX (grabbable object) and table
 	arm_controller::manipulateGoal goal = arm_controller_helper_->clearObstaclesMessage();
 	// We do not care about the arm here
	if ( not armAction ( ArmController::Arms::NONE, goal) )
		ROS_ERROR("Could not clear the scene of obstacles");

	PoseStamped grab;
	if ( not getGrabPose(item, grab))
		ROS_ERROR("Could not get grab pose");

    PoseStamped pre_grab;
	if ( not getPreGrabPose(item, pre_grab))
		ROS_ERROR("Could not get pre grab pose");

    PoseStamped post_grab;
	if ( not getPostGrabPose(item, post_grab))
		ROS_ERROR("Could not get post grab pose");

	vector<PoseStamped> poses;
	poses.push_back(post_grab);
	poses.push_back(grab);
	poses.push_back(pre_grab);

	ArmController::Arms arm = pickArm( poses );
	if (arm == ArmController::Arms::NONE)
	{
		result_.return_code = OUT_OF_REACH_ERROR;
		removeTransform(item_id);
		sendResult(false);
		return;
	}

	continueGrabAction(arm, item_id, parameters); 
}

void Grab::continueGrabAction( const ArmController::Arms arm, const std::string item_id, const vector<std::string> parameters )
{
	operator_gui_->message("Denken...");
	
	ROS_INFO("Grab::grabAction(continue)");
	Item item = datamanager_->get<Item>(item_id);

	PoseStamped item_pose = item.get_bounding_box().getPoseStamped();

	//! @todo MdL: Fix base_link?.
	// if ( (item.get_bounding_box().getHeigth() / 2) < MIN_GRIPPER_TABLE_DIST_DOWN )
	// {
	// 	ROS_DEBUG_NAMED(ROS_NAME, "Object center point too low, adding %fm to grabbing point", (MIN_GRIPPER_TABLE_DIST_DOWN - (item.get_bounding_box().getHeigth() / 2)));
	// 	rose_transformations::addXYZInFrame(tf_, "base_link", 0.0, 0.0, (MIN_GRIPPER_TABLE_DIST_DOWN - (item.get_bounding_box().getHeigth() / 2)), item_pose);
	// }

	updateTransform(item_id, item_pose);

	if ( not closeGripper(arm) )
	{
        ROS_ERROR_NAMED(ROS_NAME, "Grab::continueGrabAction: could not closeGripper");
        grabFailed(item, "Kon de grijper niet sluiten");
        // grabFailed(item, "Could not close gripper");
		return;
	}

	operator_gui_->message("Arm beweging plannen");
	if ( not preGrabStatic(arm, item) )
	{
        ROS_ERROR_NAMED(ROS_NAME, "Grab::continueGrabAction: could not preGrabStatic");
        // grabFailed(item, "Could not move to pre-grab");
        grabFailed(item, "Kon geen arm beweging plannen");
		return;
	}

	// if ( not preGrabDynamic(arm, item) )
	// {
 //        ROS_ERROR_NAMED(ROS_NAME, "Grab::continueGrabAction: could not preGrabDynamic");
	// 	grabFailed(item);
	// 	return;
	// }

	if ( not openGripper(arm) )
	{
        ROS_ERROR_NAMED(ROS_NAME, "Grab::continueGrabAction: could not openGripper");
        grabFailed(item, "Kon de grijper niet openen");
        // grabFailed(item, "Could not open gripper");
		return;
	}

	if ( not moveTowardsItem(arm, item) )
	{
        ROS_ERROR_NAMED(ROS_NAME, "Grab::continueGrabAction: could not moveTowardsItem");
        grabFailed(item, "Kon de arm niet bewegen");
        // grabFailed(item, "Could not move to item");
		return;
	}

	// sleep for one second
    // ros::Duration(1.0).sleep();

	if ( not closeGripper(arm) )
	{
        ROS_ERROR_NAMED(ROS_NAME, "Grab::continueGrabAction: could not closeGripper");
        grabFailed(item, "Kon de grijper niet sluiten");
        // grabFailed(item, "Could not close gripper");
		return;
	}

	// sleep for one second
    // ros::Duration(1.0).sleep();

    if ( not arm_controller_helper_->attachItem(arm, item_id))
    {
        ROS_ERROR_NAMED(ROS_NAME, "Grab::continueGrabAction: could not attachItem");
        // grabFailed(item, "Could not remember in which arm I'm holding the item");
        grabFailed(item, "Could not remember in which arm I'm holding the item");
        return;
    }

    // sleep for one second
    // ros::Duration(1.0).sleep();

	operator_gui_->message("Gripper omhoog bewegen");
	ROS_INFO_NAMED(ROS_NAME, "Going to post grab");
    if ( not moveGripperUp(arm, POST_GRAB_Z) )
    {
        ROS_ERROR_NAMED(ROS_NAME, "Grab::continueGrabAction: could not moveGripperUp");
        // grabFailed(item, "Could not move gripper up");
        grabFailed(item, "Kon de gripper niet omhoog bewegen");
		return;
    }
	
	// sleep for one second
    // ros::Duration(1.0).sleep();

	if ( not setTable(item) )
	{
        ROS_ERROR_NAMED(ROS_NAME, "Grab::continueGrabAction: could not setTable");
        grabFailed(item, "Could not remove table for arm planning");
		return;
	}

	// All actions were successful
    sendResultAndCleanup(true, item, "Grabbed item");
}

void Grab::grabFailed( Item item, std::string message)
{
    sendResultAndCleanup(false, item, message);
}

void Grab::cleanup( Item item )
{
	std::string item_id = item.get_id();

//	removeBoundingBoxOfItemFromDatabase(item); //! @todo MdL: Temporary implementation, keeps the db clean
	removeTransform(item_id);

	arm_controller::manipulateGoal goal = arm_controller_helper_->clearObstaclesMessage();
	ArmController::Arms arm = ArmController::Arms::NONE; // Do not care about arm
	armAction ( arm, goal);
}

void Grab::sendResultAndCleanup( bool result, Item item, string message )
{
    sendResult(result, message);
	cleanup(item);
}
