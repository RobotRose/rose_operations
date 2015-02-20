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
#include "move_to/move_to.hpp"

MoveTo::MoveTo( std::string name, ros::NodeHandle n )
	: OperationBaseClass (name, n)
{
	smc_->addClient<move_base_msgs::MoveBaseAction>("move_base_smc", 
		boost::bind(&MoveTo::CB_moveSuccess, this, _1, _2),
		boost::bind(&MoveTo::CB_moveFail, this, _1, _2),
		NULL, NULL);

	startOperation();
}

MoveTo::~MoveTo()
{

}

void MoveTo::CB_moveSuccess( const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result )
{
	result_.return_code = SUCCESS;
	sendResult( true, "Eindpunt bereikt" );
}

void MoveTo::CB_moveFail( const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result )
{
	result_.return_code = UNKNOWN_ERROR;
	sendResult( false, "Kon het eindpunt niet bereiken" );	
}

bool MoveTo::checkWaypoint ( const Waypoint waypoint  )
{
	return waypoint.isSet();
}

void MoveTo::executeWaypointAction( const std::string item_id, const std::vector<std::string> parameters )
{
	Waypoint waypoint = datamanager_->get<Waypoint>(item_id);
	ROS_INFO("Waypoint: %s", waypoint.get_name().c_str());

	if ( checkWaypoint(waypoint) )
		moveToLocation( waypoint );
	else 
	{
		ROS_INFO("No location set.(1)");
		operator_gui_->action("Selecteer locatie");
		getParameter(item_id, PARAMETER_REQUEST::MAP_LOCATION, boost::bind(&MoveTo::continueWaypointAction, this, _1, _2));
	}
}

void MoveTo::continueWaypointAction( const std::string item_id, const std::vector<std::string> parameters )
{
	Waypoint waypoint = datamanager_->get<Waypoint>(item_id);

	if ( checkWaypoint(waypoint) )
	{
		moveToLocation( waypoint );
	}	
	else 
	{
		ROS_INFO("No location set.(2)");
		result_.return_code = MISSING_PARAMETER_ERROR;
		sendResult( false );
	}
}

void MoveTo::executeItemAction( const std::string item_id, const std::vector<std::string> parameters )
{
	Item item = datamanager_->get<Item>(item_id);

	ROS_INFO("Item: %s", item.get_name().c_str());

	if (!item.get_bounding_box().isSet())
		getParameter(item_id, PARAMETER_REQUEST::BOUNDING_BOX, boost::bind(&MoveTo::continueItemAction, this, _1, _2));
	else
		moveToPointNearItem(item);
}

void MoveTo::continueItemAction( const std::string item_id, const std::vector<std::string> parameters )
{
	Item item = datamanager_->get<Item>(item_id);

	ROS_INFO("Item: %s", item.get_name().c_str());

	if (!item.get_bounding_box().isSet())
	{
		ROS_INFO("move_to: missing parameter for item");
		result_.return_code = MISSING_PARAMETER_ERROR;
		sendResult( false );
	}
	else
		moveToPointNearItem(item);
}

void MoveTo::moveToPointNearItem( Item item )
{
	//! @todo MdL: Has to be reimplemented with the pose_explorer for instance.
	// tf::StampedTransform transform;
    
 //    try
 //    {
	// 	tf_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
 //    }
 //    catch (tf::TransformException ex)
 //    {
	// 	ROS_ERROR("%s",ex.what());
	// 	sendResult( false );
	// 	return;
 //    }
	
	// geometry_msgs::Point rose_location;	
	// geometry_msgs::Point item_bounding_box; 

	// rose_location.x 		= transform.getOrigin().x();
	// rose_location.y 		= transform.getOrigin().y();		
	// item_bounding_box 		= item.get_bounding_box().getCenter();
	
	// double x_difference 	= rose_location.x - item_bounding_box.x;
	// double y_difference 	= rose_location.y - item_bounding_box.y;

	// rose_geometry::setVectorLengthXY( &x_difference, &y_difference, 0.79 );

	// double angle 			= rose_geometry::getAngle(rose_location, item_bounding_box);

	// Orientation orientation	= Orientation( -angle );
	// Location location 		= Location( item_bounding_box.x + x_difference, item_bounding_box.y + y_difference);

	// moveToLocation ( location, orientation );	

	// BoundingBox empty_bb;
	// item.set_bounding_box(empty_bb);

	// datamanager_->store<Item>(item);	
}	

void MoveTo::moveToLocation( const Waypoint waypoint )
{
	operator_gui_->message("Rijden...");
	move_base_msgs::MoveBaseGoal goal;
	geometry_msgs::PoseStamped 	 waypoint_pose = waypoint.getPoseStamped();

	goal.target_pose = waypoint_pose;

	if( not smc_->sendGoal<move_base_msgs::MoveBaseAction>(goal, "move_base_smc", 1.0) )
	{
		ROS_INFO("Could not send goal to move_base_smc, aborting moveto operation.");
		smc_->abort();
	}
	else
	{
		ROS_INFO("Goal send goal to move_base_smc, waiting for result.");
		smc_->waitForResult("move_base_smc", ros::Duration(0.0));
	}

	if ( waypoint.is_temp() )
		datamanager_->deleteObject<Waypoint>(waypoint);
}