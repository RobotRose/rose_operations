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
    smc_->addClient<rose_arm_controller_msgs::set_positionAction>("arm_controller/position");
    smc_->addClient<rose_arm_controller_msgs::set_gripper_widthAction>("arm_controller/gripper_width");
    smc_->addClient<rose_arm_controller_msgs::move_to_tfAction>("arm_visual_servoing");

    startOperation();
}

Grab::~Grab()
{
    
}

void Grab::CB_serverCancel( SMC* smc )
{
    operator_gui_->message("Cancel ontvangen");
}

// Goal received
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
        executeGrabItem( item_id, parameters );
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
        executeGrabItem ( item_id, parameters );
}

// Handy poses

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
    return getPoseRelativeToGrab( item, pose, 0.0, 0.0, 0.0 ); 
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

bool Grab::preGrab( const std::string arm_name, Item item )
{
    ROS_DEBUG_NAMED(ROS_NAME, "Pre grab");

    geometry_msgs::PoseStamped pre_grab;

    if ( not getPreGrabPose(item, pre_grab))
        return false;

    rose_arm_controller_msgs::set_positionGoal position_goal;

    position_goal.arm           = arm_name;
    position_goal.required_pose = pre_grab;

    if ( not smc_->sendGoal<rose_arm_controller_msgs::set_positionAction>(position_goal, "arm_controller/position"))
    {
        ROS_ERROR("Could not send goal to the arm controller");
        return false;
    }

    if ( not smc_->waitForSuccess("arm_controller/position", ros::Duration(10.0)) ) // Wait 10s max
        return false;
    else 
        return true;

    return true;
}

bool Grab::grab( const std::string arm_name, Item item )
{
    ROS_DEBUG_NAMED(ROS_NAME, "grab");

    moveTowardsItem(arm_name, item);

    return true;
}

bool Grab::postGrab( const std::string arm_name, Item item )
{
    ROS_DEBUG_NAMED(ROS_NAME, "Post grab");

    geometry_msgs::PoseStamped post_grab;

    if ( not getPostGrabPose(item, post_grab))
        return false;

    rose_arm_controller_msgs::set_positionGoal position_goal;

    position_goal.arm           = arm_name;
    position_goal.required_pose = post_grab;

    if ( not smc_->sendGoal<rose_arm_controller_msgs::set_positionAction>(position_goal, "arm_controller/position"))
    {
        ROS_ERROR("Could not send goal to the arm controller");
        return false;
    }

    if ( not smc_->waitForSuccess("arm_controller/position", ros::Duration(10.0)) ) // Wait 10s max
        return false;
    else 
        return true;

    return true;
}

//! @todo MdL: Test basic 'visual servoing'.
//! Replaces grab
bool Grab::moveTowardsItem( const std::string arm_name, Item item )
{
    std::string item_id = item.get_id();

    ROS_DEBUG_NAMED(ROS_NAME, "Move towards grab");
    operator_gui_->message("Beweging richting item");

    // Visual servoing
    rose_arm_controller_msgs::move_to_tfGoal servoing_goal;

    servoing_goal.arm_name          = arm_name;
    servoing_goal.tf_name           = item_id;
    // First move in xy (arm frame)
    servoing_goal.max_xy_error  = 0.018;

    if ( not smc_->sendGoal<rose_arm_controller_msgs::move_to_tfAction>(servoing_goal, "arm_controller/positionarm_visual_servoing"))
    {
        ROS_ERROR("Could not send visual servoing");
        return false;
    }

    if ( not smc_->waitForSuccess("arm_visual_servoing", ros::Duration(10.0)) ) // Wait 40s max
        return false;
    else 
        return true;
}

// Grab sequence
void Grab::executeGrabItem( const std::string item_id, const vector<std::string> parameters )
{
    std::string arm_name = "mico";

    operator_gui_->message("Grabbing item");

    // Get item details
    Item item      = datamanager_->get<Item>(item_id);
    geometry_msgs::PoseStamped item_pose = item.get_bounding_box().getPoseStamped();
    
    createTransform(item_id, item_pose);

    operator_gui_->message("Geting arm to first pose");
    // Go to pre grab
    if ( not preGrab(arm_name, item) )
    {
        ROS_ERROR("Could not pre grab");
        grabFailed(item, "Pre grab was not possible");
        return;
    }

    operator_gui_->message("Going to grab");
    // Grab with visual servoing
    if ( not grab(arm_name, item) )
    {
        ROS_ERROR("Grab failed");
        grabFailed(item, "Moving to the item was not possible");
        return;
    }

    // operator_gui_->message("Moving arm upwards");
    // Pre grab
    // if ( not postGrab(arm_name, item) )
    // {
    //  ROS_ERROR("Post grab failed");
    //  grabFailed(item, "Could not move gripper up");
    //  return;
    // }

    //! @todo MdL [IMPL]: post grab.
    operator_gui_->message("Please use the joystick to move the gripper up");

    grabSuccess(item);
}

void Grab::grabSuccess( Item item )
{
    sendResultAndCleanup(true, item, "Grabbed item");
}

void Grab::grabFailed( Item item, std::string message)
{
    sendResultAndCleanup(false, item, message);
}

void Grab::cleanup( Item item )
{
    std::string item_id = item.get_id();

//  removeBoundingBoxOfItemFromDatabase(item); //! @todo MdL: Temporary implementation, keeps the db clean
    removeTransform(item_id);
}

void Grab::sendResultAndCleanup( bool result, Item item, string message )
{
    sendResult(result, message);
    cleanup(item);
}
