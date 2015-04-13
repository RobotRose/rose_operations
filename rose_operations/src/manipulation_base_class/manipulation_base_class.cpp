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

	gaze_client_ = new GazeClient("gaze_controller", true); 
}

ManipulationBaseClass::~ManipulationBaseClass()
{
	
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
    rose_gaze_controller::LookAtGoal goal;

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

