/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
* Author: Mathijs de Langen
* Date  : 2014/02/14
*     - File created.
*
* Description:
* description
* 
***********************************************************************************/
#include "place/place.hpp"

Place::Place( std::string name, ros::NodeHandle n )
    : ManipulationBaseClass(name, n)
    , operator_defined_waypoint("_klik_in_camera")
{
    ROS_DEBUG_NAMED(ROS_NAME, "");
}

Place::~Place()
{
	
}

void Place::CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
	
}

void Place::CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
    operator_gui_->warn("Arms failed");
}

void Place::executeItemAction( const std::string item_id, const std::vector<std::string> parameters )
{
    ROS_INFO_NAMED(ROS_NAME, "Place::executeItemAction: item_id = %s", item_id.c_str());

    Waypoint waypoint = Waypoint();

    /******* DEBUG INFO **********/
    if(parameters.size() > 0)
    {
        std::string waypoint_id = parameters.at(0);
        ROS_INFO_NAMED(ROS_NAME, "Place::executeItemAction: waypoint_id = %s", waypoint_id.c_str());
        waypoint = datamanager_->get<Waypoint>(waypoint_id);
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction waypoint ID: %s", waypoint.get_id().c_str());
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction waypoint name: %s", waypoint.get_name().c_str());
    }
    else
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction did not receive any parameters");
    }

    ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction getting item for id %s", item_id.c_str());
    Item item = datamanager_->get<Item>(item_id);
    ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction got item for id %s", item_id.c_str());

    // bounding box of object in hand should be known
    if ( not item.get_bounding_box().isSet())
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction bounding box for item_id %s not isSet", item_id.c_str());
        result_.return_code = MISSING_PARAMETER_ERROR;
        sendResult( false, "Ik weet niet hoe groot dit item is. Plaats manueel" );
        return;
    }

    // If the operator selected the special OPERATOR_DEFINED_WAYPOINT, ask for the operator to define the waypoint
    std::string escaped(operator_defined_waypoint);
    std::replace( escaped.begin(), escaped.end(), '_', ' ');
    std::size_t found = waypoint.get_name().find(escaped);
    ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction Index of %s in %s is %i", operator_defined_waypoint.c_str(), waypoint.get_name().c_str(), (int)found);
    if(found != std::string::npos)
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction No waypoint specified, so requesting one");
        operator_gui_->action("Select a place region");

//        Waypoint stored_waypoint;
//        stored_waypoint = datamanager_->store<Waypoint>(waypoint); //A new waypoint is created because we want to store the selected location somewhere in the DB.
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction asking parameter manager to ask for a waypoint to be stored at %s", operator_defined_waypoint.c_str());
        getParameter( waypoint.get_id(), PARAMETER_REQUEST::PLACE_LOCATION, boost::bind(&Place::continueItemAndWaypoint, this, _1, waypoint.get_id(), _2));
        return;
    }
    else
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction items and parameters fully defined.");
        prePlaceAction(item_id, waypoint, parameters);
//	if ( not boost::starts_with(parameters.at(0), Waypoint::IDENTIFIER))
//	{
//		// We need a waypoint to place
//		result_.return_code = UNKNOWN_ERROR;
//		sendResult( false );
//		return;
//	}

//	Waypoint waypoint = datamanager_->get<Waypoint>(parameters.at(0));

//	if ( waypoint.get_id() == "waypoint1" )
//	{
//		staticPlacementAction( item_id, waypoint );
//		return;
//	}

//	if ( not waypoint.isSet() )
//	{
//        getParameter( parameters.at(0), PARAMETER_REQUEST::PLACE_LOCATION, boost::bind(&Place::continueItemAction, this, _1, _2));
//		return;
//	}


    }
}

void Place::executeMultiItemAction( const std::vector<std::string> item_ids, const std::vector<std::string> parameters )
{
	ROS_INFO_NAMED(ROS_NAME, "Place::executeItemAction");

    std::string item_id = item_ids.at(0);
    std::string waypoint_id = item_ids.at(1);
    ROS_INFO_NAMED(ROS_NAME, "Place::executeItemAction: item_id = %s", item_id.c_str());
    ROS_INFO_NAMED(ROS_NAME, "Place::executeItemAction: waypoint_id = %s", waypoint_id.c_str());

    Waypoint waypoint = Waypoint();
    /******* DEBUG INFO **********/
    if(item_ids.size() > 0)
    {
        waypoint = datamanager_->get<Waypoint>(waypoint_id);
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction waypoint ID: %s", waypoint.get_id().c_str());
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction waypoint name: %s", waypoint.get_name().c_str());
    }
    else
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction did not receive any parameters");
    }

    ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction getting item for id %s", item_id.c_str());
	Item item = datamanager_->get<Item>(item_id);
    ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction got item for id %s", item_id.c_str());

	// bounding box of object in hand should be known
	if ( not item.get_bounding_box().isSet())
	{
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction bounding box for item_id %s not isSet", item_id.c_str());
		result_.return_code = MISSING_PARAMETER_ERROR;
		sendResult( false );
		return;
    }

    // If the operator selected the special OPERATOR_DEFINED_WAYPOINT, ask for the operator to define the waypoint
    std::string escaped(operator_defined_waypoint);
    std::replace( escaped.begin(), escaped.end(), '_', ' ');
    std::size_t found = waypoint.get_name().find(escaped);
    ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction Index of %s in %s is %i", operator_defined_waypoint.c_str(), waypoint.get_name().c_str(), (int)found);
    if(found != std::string::npos)
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction No waypoint specified, so requesting one");
        operator_gui_->action("Select a place region");

//        Waypoint stored_waypoint;
//        stored_waypoint = datamanager_->store<Waypoint>(waypoint); //A new waypoint is created because we want to store the selected location somewhere in the DB.
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction asking parameter manager to ask for a waypoint to be stored at %s", operator_defined_waypoint.c_str());
        getParameter( waypoint.get_id(), PARAMETER_REQUEST::PLACE_LOCATION, boost::bind(&Place::continueItemAndWaypoint, this, _1, waypoint.get_id(), _2));
        return;
    }
    else
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Place::executeItemAction items and parameters fully defined.");
        prePlaceAction(item_id, waypoint, parameters);
//	if ( not boost::starts_with(parameters.at(0), Waypoint::IDENTIFIER))
//	{
//		// We need a waypoint to place
//		result_.return_code = UNKNOWN_ERROR;
//		sendResult( false );
//		return;
//	}

//	Waypoint waypoint = datamanager_->get<Waypoint>(parameters.at(0));

//	if ( waypoint.get_id() == "waypoint1" )
//	{
//		staticPlacementAction( item_id, waypoint );
//		return;
//	}

//	if ( not waypoint.isSet() )
//	{
//        getParameter( parameters.at(0), PARAMETER_REQUEST::PLACE_LOCATION, boost::bind(&Place::continueItemAction, this, _1, _2));
//		return;
//	}


    }
}

void Place::continueItemAndWaypoint( const std::string item_id, const std::string waypoint_id, const vector<std::string> parameters )
{
    ROS_INFO("Place::continueItemAndWaypoint with item_id %s and waypoint_id %s", item_id.c_str(), waypoint_id.c_str());
    Item item = datamanager_->get<Item>(item_id);
    Waypoint place_location = datamanager_->get<Waypoint>(waypoint_id);

    operator_gui_->message("Thanks!"); //The operator gave a waypoint, thanks for the help.

    /******* DEBUG INFO **********/
    if(parameters.size() > 0)
    {
        for(auto param : parameters)
        {
            ROS_DEBUG_NAMED(ROS_NAME, "Place::continueItemAndWaypoint parameter: %s", param.c_str());
        }
    }
    else
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Place::continueItemAndWaypoint did not receive any parameters");
    }

//    if (!item.get_bounding_box().isSet())
//    {
//        result_.return_code = MISSING_PARAMETER_ERROR;
//        sendResult( false );
//    }
//    else
//    {
//        Waypoint waypoint = datamanager_->get<Waypoint>(parameters.at(0));

//        if ( waypoint.get_id() == "waypoint1" )
//        {
//            staticPlacementAction( item_id, waypoint );
//            return;
//        }

//        if ( not waypoint.isSet() )
//        {
//            getParameter( parameters.at(0), PARAMETER_REQUEST::PLACE_LOCATION, boost::bind(&Place::continueItemAction, this, _1, _2));
//            return;
//        }
//    }
    prePlaceAction ( item_id, place_location, parameters );
}

void Place::prePlaceAction( const std::string item_id, const Waypoint place_location, const vector<std::string> parameters )
{
    ROS_DEBUG_NAMED(ROS_NAME, "Place::prePlaceAction");
    operator_gui_->message("Thinking...");

    // Get item and bounding box (do this first, because transform creation/syncing takes some time)
    Item item 	   = datamanager_->get<Item>(item_id);
    ArmController::Arms arm = arm_controller_helper_->armHoldingItem(item_id);
    if((int)arm == 0 || (int)arm == -1) //! @todo LvB: will not work when using rhe right arm too. We need to give ArmController::Arms::None an index too.
    {
        sendResult( false, "Ik weet niet in welke hand ik dit item heb. Plaats manueel" );
        return;
    }

    geometry_msgs::PoseStamped pose = place_location.getPoseStamped();
    ROS_DEBUG_NAMED(ROS_NAME, "Place::prePlaceAction: creating transform place_%s at (%.3f, %.3f, %.3f)x(%.3f, %.3f, %.3f, %.3f) in %s",
                    item_id.c_str(), pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w,
                    pose.header.frame_id.c_str());

    bool success = rose_transformations::transformToFrame(tf_, arm_controller_helper_->getFrameFor(arm), pose);
    ROS_DEBUG_NAMED(ROS_NAME, "Place::prePlaceAction: creating transform place_%s at (%.3f, %.3f, %.3f)x(%.3f, %.3f, %.3f, %.3f) in %s",
                    item_id.c_str(), pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w,
                    pose.header.frame_id.c_str());

    createTransform("place_"+item_id, place_location.getPoseStamped());

    ROS_DEBUG_NAMED(ROS_NAME, "item_id: %s", item_id.c_str()); //, place_location.get_name().c_str()

    // Clear other obstacles and set the BOX (grabbable object) and table
    arm_controller::manipulateGoal goal = arm_controller_helper_->clearObstaclesMessage();
    // We do not care about the arm here
    if ( not armAction ( ArmController::Arms::NONE, goal) )
        ROS_ERROR("Could not clear the scene of obstacles");

    PoseStamped pre_place;
    if ( not getPrePlacePose(item, place_location, pre_place))
        ROS_ERROR("Could not get pre_place pose");

    PoseStamped place;
    if ( not getPlacePose(item, place_location, place))
        ROS_ERROR("Could not get place pose");

    insertTableAtPlace(arm, place_location.getPoseStamped());

    vector<PoseStamped> poses;
    poses.push_back(pre_place);
    poses.push_back(place);

//    ROS_DEBUG_NAMED(ROS_NAME, "Getting number of reachable poses");
//    int nrReachable = getNrOfReachablePoints(arm, poses);

//    if (false)//(nrReachable < (int)poses.size())
//    {
//        result_.return_code = OUT_OF_REACH_ERROR;
//        operator_gui_->warn("Position out of reach. Move closer");
//        sendResultAndCleanup(false, item, place_location);
//        return;
//    }

    ROS_DEBUG_NAMED(ROS_NAME, "Continueing place");
    continuePlaceAction(arm, item_id, place_location, parameters);
}

bool Place::insertTableAtPlace(const ArmController::Arms arm, PoseStamped place)
{
    PoseStamped tablePose = PoseStamped(place);
    bool transformSuccess = rose_transformations::transformToLatestFrame(tf_, "/base_link", tablePose);
    if(not transformSuccess)
    {
        ROS_ERROR_NAMED(ROS_NAME, "Place::insertTableAtPlace could not transformToLatestFrame from %s to %s", tablePose.header.frame_id.c_str(), "/base_link");
        return false;
    }
    tablePose.pose.position.z -= TABLE_HEIGHT/2;
    tablePose.pose.orientation = rose_conversions::RPYToQuaterion(0,0,0);
    ROS_DEBUG_NAMED(ROS_NAME, "Place::insertTableAtPlace; table_pose = (%3.3f, %3.3f, %3.3f in %s)",
                    tablePose.pose.position.x, tablePose.pose.position.y, tablePose.pose.position.z, tablePose.header.frame_id.c_str());

    obstacles.insert(std::pair<ArmController::Manipulators, geometry_msgs::PoseStamped>(ArmController::Manipulators::TABLE, tablePose));

    arm_controller::manipulateGoal obstacle_goal = arm_controller_helper_->setObstaclesMessage( obstacles );
    if ( not armAction ( arm, obstacle_goal) )
    {
        ROS_ERROR_NAMED(ROS_NAME, "Place::insertTableAtPlace could not perform armAction for obstacle insertion");
        return false;
    }
    return true;
}

bool Place::insertItemAtPlace(const ArmController::Arms arm, PoseStamped item)
{
    PoseStamped itemPose = PoseStamped(item);
    bool transformSuccess = rose_transformations::transformToLatestFrame(tf_, "/base_link", itemPose);
    if(not transformSuccess)
    {
        ROS_ERROR_NAMED(ROS_NAME, "Place::insertItemAtPlace could not transformToLatestFrame from %s to %s", itemPose.header.frame_id.c_str(), "/base_link");
        return false;
    }

    itemPose.pose.orientation = rose_conversions::RPYToQuaterion(0,0,0);
    ROS_DEBUG_NAMED(ROS_NAME, "Place::insertTableAtPlace; table_pose = (%3.3f, %3.3f, %3.3f in %s)",
                    itemPose.pose.position.x, itemPose.pose.position.y, itemPose.pose.position.z, itemPose.header.frame_id.c_str());

    obstacles.insert(std::pair<ArmController::Manipulators, geometry_msgs::PoseStamped>(ArmController::Manipulators::BOX, itemPose));

    arm_controller::manipulateGoal obstacle_goal = arm_controller_helper_->setObstaclesMessage( obstacles );
    if ( not armAction ( arm, obstacle_goal) )
    {
        ROS_ERROR_NAMED(ROS_NAME, "Place::insertItemAtPlace could not perform armAction for obstacle insertion");
        return false;
    }
    return true;
}

bool Place::getPrePlacePose( Item item, const Waypoint place_location, PoseStamped& pose )
{
    double objectHeight = item.get_bounding_box().getHeigth()/2;
    double gripper_to_center_z = item.get_bounding_box().getPoseStamped().pose.position.z;
    double total_pre_place_offset = PRE_PLACE_Z_OFFSET+objectHeight-gripper_to_center_z;

    ROS_DEBUG_NAMED(ROS_NAME, "Place::getPrePlacePose: objectHeight:            %2.3f", objectHeight);
    ROS_DEBUG_NAMED(ROS_NAME, "Place::getPrePlacePose: PRE_PLACE_Z_OFFSET:      %2.3f", PRE_PLACE_Z_OFFSET);
    ROS_DEBUG_NAMED(ROS_NAME, "Place::getPrePlacePose: gripper_to_center_z:     %2.3f", -gripper_to_center_z);
    ROS_DEBUG_NAMED(ROS_NAME, "Place::getPrePlacePose: ------------------------------ + ");
    ROS_DEBUG_NAMED(ROS_NAME, "Place::getPrePlacePose: total_pre_place_offset:  %2.3f", total_pre_place_offset);


    pose = geometry_msgs::PoseStamped(place_location.getPoseStamped());
    pose.header.stamp = ros::Time::now();
    bool success = rose_transformations::addXYZInFrameNow(tf_, "/base_link", 0, 0, total_pre_place_offset, pose);

    /* Debugging */
    PoseStamped debug_pose = PoseStamped(pose);
    if(rose_transformations::transformToFrameNow(tf_, "/base_link", debug_pose, 1.0))
    {
        ROS_DEBUG_NAMED(ROS_NAME, "prePlacePose: (%3.3f, %3.3f, %3.3f in %s)",
                        debug_pose.pose.position.x, debug_pose.pose.position.y, debug_pose.pose.position.z, debug_pose.header.frame_id.c_str());
    }
    else
    {
        ROS_DEBUG_NAMED(ROS_NAME, "prePlacePose: (%3.3f, %3.3f, %3.3f in %s)",
                     pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.header.frame_id.c_str());
    }

    return success;
}

bool Place::getPlacePose( Item item, const Waypoint place_location, PoseStamped& pose )
{
    double center_to_bottom = item.get_bounding_box().getHeigth()/2;
    double gripper_to_center_z = item.get_bounding_box().getPoseStamped().pose.position.z;
    double total_place_offset = center_to_bottom-gripper_to_center_z;

    ROS_DEBUG_NAMED(ROS_NAME, "Place::getPlacePose: center_to_bottom:        %2.3f", center_to_bottom);
    ROS_DEBUG_NAMED(ROS_NAME, "Place::getPlacePose: gripper_to_center_z:     %2.3f", -gripper_to_center_z);
    ROS_DEBUG_NAMED(ROS_NAME, "Place::getPlacePose: ------------------------------ + ");
    ROS_DEBUG_NAMED(ROS_NAME, "Place::getPlacePose: total_place_offset:      %2.3f", total_place_offset);

    pose = geometry_msgs::PoseStamped(place_location.getPoseStamped());
    pose.header.stamp = ros::Time::now();
    bool success = rose_transformations::addXYZInFrameNow(tf_, "/base_link", 0, 0, total_place_offset, pose, 10.0);

    if(not success)
    {
        ROS_ERROR_NAMED(ROS_NAME, "Place::getPlacePose could not add total_pre_place_offset=%2.3f to pose in /base_link", total_place_offset);
    }

    /* Debugging */
    PoseStamped debug_pose = PoseStamped(pose);
    if(rose_transformations::transformToFrameNow(tf_, "/base_link", debug_pose, 1.0))
    {
        ROS_DEBUG_NAMED(ROS_NAME, "placePose: (%3.3f, %3.3f, %3.3f in %s)",
                     debug_pose.pose.position.x, debug_pose.pose.position.y, debug_pose.pose.position.z, debug_pose.header.frame_id.c_str());
    }
    else
    {
        ROS_DEBUG_NAMED(ROS_NAME, "placePose: (%3.3f, %3.3f, %3.3f in %s)",
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.header.frame_id.c_str());
    }

    return success;
}

void Place::continuePlaceAction(const ArmController::Arms arm,  const std::string item_id, const Waypoint place_location, const std::vector<std::string> parameters )
{
    operator_gui_->message("Thinking...");

    ROS_DEBUG_NAMED(ROS_NAME, "Place::continuePlaceAction");
    Item item = datamanager_->get<Item>(item_id);

    updateTransform("place_"+item_id, place_location.getPoseStamped());

    ROS_DEBUG_NAMED(ROS_NAME, "Pre-placing");
    if ( not prePlace(arm, place_location, item) )
    {
        ROS_ERROR_NAMED(ROS_NAME, "Could not prePlace");
        sendResultAndCleanup(false, item, place_location, "Could not move to pre-place");
        return;
    }

    ROS_DEBUG_NAMED(ROS_NAME, "moveTowardPlacement");
    if ( not moveTowardPlacement(arm, place_location, item) )
    {
        ROS_ERROR_NAMED(ROS_NAME, "Could not moveTowardPlacement");
        sendResultAndCleanup(false, item, place_location, "Could not move to item");
        return;
    }

    ROS_DEBUG_NAMED(ROS_NAME, "openGripper");
    if ( not openGripper(arm) )
    {
        ROS_ERROR_NAMED(ROS_NAME, "Could not openGripper");
        sendResultAndCleanup(false, item, place_location, "Could not open gripper");
        return;
    }

    ROS_DEBUG_NAMED(ROS_NAME, "detachItem");
    arm_controller_helper_->detachItem(arm, item_id);
    //This is not crucial?
//    if ( not arm_controller_helper_->detachItem(arm, item_id))
//    {
//        sendResult(false);
//        return;
//    }

//    insertItemAtPlace(arm, item.get_bounding_box().getPoseStamped());

    ROS_DEBUG_NAMED(ROS_NAME, "Go up");
    //if ( not prePlace(arm, place_location, item) ) //Go back up!
    if( not moveGripperUp(arm, (item.get_bounding_box().getHeigth()/2) + POST_PLACE_Z_OFFSET, true))
    {
        ROS_ERROR_NAMED(ROS_NAME, "Could not moveGripperUp");
        sendResultAndCleanup(false, item, place_location, "Gripper could not move up");
        return;
    }

    removeTransform("place_"+item_id);
    sendResultAndCleanup(true, item, place_location, "Item placed");
}

bool Place::prePlace( const ArmController::Arms arm, const Waypoint place_location, Item item )
{
        ROS_DEBUG_NAMED(ROS_NAME, "Place::prePlace");
        operator_gui_->message("Moving arm");

        PoseStamped pre_place;
        if ( not getPrePlacePose(item, place_location, pre_place))
            return false;

        ROS_DEBUG_NAMED(ROS_NAME, "Pre place at x: %f, y: %f, z: %f in frame %s",
                        pre_place.pose.position.x, pre_place.pose.position.y, pre_place.pose.position.z, pre_place.header.frame_id.c_str());

//        ROS_ERROR_NAMED(ROS_NAME, "Press the any key to continue to the preplace-position");
//        std::cin.ignore();
//        ROS_DEBUG_NAMED(ROS_NAME, "Continuing to the preplace-position");

        // Execute arm action
        arm_controller::manipulateGoal goal = arm_controller_helper_->moveToPoseMessage( arm, pre_place, true );
        if ( not armAction ( arm, goal) )
            return false;

        return true;
}

bool Place::place( const ArmController::Arms arm, const Waypoint place_location, Item item )
{
    ROS_DEBUG_NAMED(ROS_NAME, "Place::place");
    operator_gui_->message("Moving arm");

    // Remove BOX before grab action. Otherwise the arm will collide in the simulator and not move.
    vector<ArmController::Manipulators> obstacles;
    obstacles.push_back(ArmController::Manipulators::BOX);
    arm_controller::manipulateGoal goal = arm_controller_helper_->removeObstaclesMessage(obstacles);
    if ( not armAction ( arm, goal) )
        return false;

    PoseStamped place;
    if ( not getPlacePose(item, place_location, place))
        return false;

    ROS_DEBUG_NAMED(ROS_NAME, "Grab at x: %f, y: %f, z: %f in frame %s", place.pose.position.x, place.pose.position.y, place.pose.position.z, place.header.frame_id.c_str());

    goal = arm_controller_helper_->moveToPoseMessage( arm, place, true );
    if ( not armAction ( arm, goal) )
        return false;

    return true;
}

bool Place::moveTowardPlacement(const ArmController::Arms arm, const Waypoint place_location, Item item)
{
    PoseStamped placement;
    getPlacePose(item, place_location, placement);

    if ( not rose_transformations::transformToFrame(tf_, arm_controller_helper_->armToString(arm)+"_observed_gripper_tip", placement ,5))
    {
        ROS_ERROR("Could not determine error");
        operator_gui_->message("Could not find gripper marker");

        return place(arm, place_location, item);
    }

    arm_controller::manipulateGoal arm_goal = arm_controller_helper_->moveToRelativePoseMessage( arm, placement, true );

    //arm_goals are in the arms-frame
    double y_temp = arm_goal.required_pose.position.y;
    arm_goal.required_pose.position.y = 0.0;

    ROS_DEBUG_NAMED(ROS_NAME, "Place::moveTowardPlacement placement: (%3.3f, %3.3f, %3.3f in %s)",
                placement.pose.position.x, placement.pose.position.y, placement.pose.position.z, placement.header.frame_id.c_str());
    ROS_DEBUG_NAMED(ROS_NAME, "Place::moveTowardPlacement arm_goal: (%3.3f, %3.3f, %3.3f)",
                arm_goal.required_pose.position.x, arm_goal.required_pose.position.y, arm_goal.required_pose.position.z);

//    ROS_ERROR_NAMED(ROS_NAME, "Press the any key to continue with error correction step 1");
//    std::cin.ignore();
//    ROS_DEBUG_NAMED(ROS_NAME, "Continuing with error correction step 1");

    // Move towards new position (1)
    if ( not armAction ( arm, arm_goal) )
    {
        ROS_ERROR("Could not move to error correction (step 1), going to place without correction");
        return place(arm, place_location, item);
    }

//    ROS_ERROR_NAMED(ROS_NAME, "Press the any key to continue with error correction step 2");
//    std::cin.ignore();
//    ROS_DEBUG_NAMED(ROS_NAME, "Continuing with error correction step 2");

    arm_goal.required_pose.position.x = 0.0;
    arm_goal.required_pose.position.y = y_temp;
    arm_goal.required_pose.position.z = 0.0;

    ROS_DEBUG_NAMED(ROS_NAME, "Place::moveTowardPlacement arm_goal: (%3.3f, %3.3f, %3.3f)",
                arm_goal.required_pose.position.x, arm_goal.required_pose.position.y, arm_goal.required_pose.position.z);

    // Move towards new position (2)
    if ( not armAction ( arm, arm_goal) )
    {
        ROS_ERROR("Could not move to error correction (step 2), going to place without correction");
        return place(arm, place_location, item);
    }
    return true;
}

void Place::placeAction( const std::string item_id, const Waypoint place_location )
{
	//! @todo MdL: Place operation fix.
	ROS_INFO("Place::placeAction");

	Item item = datamanager_->get<Item>(item_id);
	// createPlaceLocationTF(item, place_location);

	//temp
	// BoundingBox bb 			= item.get_bounding_box();
	// double height_offset 	= bb.getHeigth() / 2;

	// std::vector<std::string> arms = arm_controller_helper_->getAvailableArms();
	// std::string arm = arms.at(0); //! @todo MdL: Pick correct arm

	// ROS_DEBUG_NAMED(ROS_NAME, "Place item at x: %f, y: %f, z: %f", place_location.get_location().get_x(), place_location.get_location().get_y(), place_location.get_location().get_z());

	// geometry_msgs::Pose zero_pose;
	// zero_pose.position.x 	= 0;
	// zero_pose.position.y 	= 0;
	// zero_pose.position.z 	= 0;
	// double roll 			= 0;
	// double pitch 			= 0;
	// double yaw				= 0;
	// zero_pose.orientation 	= rose_conversions::RPYToQuaterion(roll, pitch, yaw);

	// geometry_msgs::Pose pre_place = zero_pose;
 //    pre_place.position.z    += 0.10;
 //    pre_place = getBaselinkCoordinatesRelativeToObject(arm, pre_place);
 //    ROS_DEBUG_NAMED(ROS_NAME, "Pre place at x: %f, y: %f, z: %f", pre_place.position.x, pre_place.position.y, pre_place.position.z);

 //    geometry_msgs::Pose place = zero_pose;
 //    place = getBaselinkCoordinatesRelativeToObject(arm, place);
 //    ROS_DEBUG_NAMED(ROS_NAME, "Place at x: %f, y: %f, z: %f", place.position.x, place.position.y, place.position.z);

	// geometry_msgs::Pose post_place = zero_pose;
	// post_place = getBaselinkCoordinatesRelativeToObject(arm, post_place);
	// post_place.position.z    += height_offset + 0.05;
	// ROS_DEBUG_NAMED(ROS_NAME, "Post place at x: %f, y: %f, z: %f", post_place.position.x, post_place.position.y, post_place.position.z);

 //    arm_controller::manipulateGoal goal;

	// goal = arm_controller_helper_->moveToPoseMessage( arm, pre_place, false );
	// if ( not armAction ( arm, goal) )
	// 	return;

	// goal = arm_controller_helper_->moveToPoseMessage( arm, pre_place, true );
	// if ( not armAction ( arm, goal) )
	// 	return;

 //    goal = arm_controller_helper_->moveToPoseMessage( arm, place, true );
	// if ( not armAction ( arm, goal) )
	// 	return;

 //    goal = arm_controller_helper_->openGripperMessage( arm );
	// if ( not armAction ( arm, goal) )
	// 	return;

 //    goal = arm_controller_helper_->moveToPoseMessage( arm, post_place, true );
	// if ( not armAction ( arm, goal) )
	// 	return;

	// removeBoundingBoxOfItemFromDatabase(item);
	// removeLocationFromDatabase(place_location);
	sendResult( true );
}

void Place::removeBoundingBoxOfItemFromDatabase( Item item )
{
	item.get_bounding_box().unset();
	datamanager_->store<Item>(item);
}

void Place::removeLocationFromDatabase( Waypoint waypoint )
{
	waypoint.unset();
    datamanager_->deleteObject<Waypoint>(waypoint);
}

void Place::cleanup( Item item, Waypoint waypoint)
{
    std::string item_id = item.get_id();

    removeBoundingBoxOfItemFromDatabase(item); //! @todo MdL: Temporary implementation, keeps the db clean

    //The DB contains a special waypoint with ID "operator_defined_waypoint". When used, to operator must select a waypoint in the camera image.
    //This waypoint can never be '.set()'
    if(waypoint.get_name().find(operator_defined_waypoint) != std::string::npos)
    {
        //Delete the waypoint if it is defined in a camera frame. These move so there is no point in storing them
        std::size_t found = waypoint.getPoseStamped().header.frame_id.find("camera");
        if (found!=std::string::npos)
        {
            removeLocationFromDatabase(waypoint);
        }
    }
    else
    {
//        waypoint.unset();
//        datamanager_->store(waypoint);
    }

    removeTransform("place_"+item_id);

    arm_controller::manipulateGoal goal = arm_controller_helper_->clearObstaclesMessage();
    ArmController::Arms arm = ArmController::Arms::NONE; // Do not care about arm
    armAction ( arm, goal);
    ROS_DEBUG_NAMED(ROS_NAME, "Removed table obstacle");
}

void Place::sendResultAndCleanup( bool result, Item item, Waypoint waypoint, std::string message )
{
    sendResult(result, message);
    cleanup(item, waypoint);
}
