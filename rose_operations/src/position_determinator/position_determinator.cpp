/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/07/22
* 		- File created.
*
* Description:
*	This node determines the 'best' pose for the robot to do a certain operation on 
*	an item. It stores this pose as a waypoint in the database to be used.
* 
***********************************************************************************/
#include "position_determinator/position_determinator.hpp"

PositionDeterminator::PositionDeterminator( std::string name, ros::NodeHandle n )
	: OperationBaseClass (name, n)
{
	reachable_position_service_ = n_.serviceClient<rose_pose_explorer::reachable_poses>("/pose_explorer/calculate_reachability");

	best_pose_pub_ 				    = n_.advertise<visualization_msgs::Marker>( name_  + "/best_pose", 1);
	best_pose_corrected_pub_        = n_.advertise<visualization_msgs::Marker>( name_  + "/best_pose_corrected", 1);
	reachable_points_before_filter_	= n_.advertise<MarkerArray>( name_  + "/reachable_points_before_filter", 1);
	reachable_points_after_filter_ 	= n_.advertise<MarkerArray>( name_  + "/reachable_points_after_filter", 1);

	startOperation();
}

PositionDeterminator::~PositionDeterminator()
{
	
}

void PositionDeterminator::executeWaypointAction( const std::string item_id, const std::vector<std::string> parameters )
{
	Waypoint waypoint = datamanager_->get<Waypoint>(item_id);

	if (!waypoint.isSet())
	{
		operator_gui_->action("Selecteer locatie");
		getParameter(item_id, PARAMETER_REQUEST::PLACE_LOCATION, boost::bind(&PositionDeterminator::continueWaypointAction, this, _1, _2));
	}
	else
		waypointAction(waypoint, parameters);
}

void PositionDeterminator::continueWaypointAction( const std::string item_id, const std::vector<std::string> parameters )
{
	Waypoint waypoint = datamanager_->get<Waypoint>(item_id);

	ROS_INFO("Waypoint: %s", waypoint.get_name().c_str());

	if (!waypoint.isSet())
	{
		result_.return_code = MISSING_PARAMETER_ERROR;
		sendResult( false );
	}
	else
		waypointAction(waypoint, parameters);
}

void PositionDeterminator::waypointAction( const Waypoint waypoint, const std::vector<std::string> parameters )
{
	ROS_DEBUG_NAMED(ROS_NAME, "PositionDeterminator::itemAction");
	//! @todo MdL: Check if they are available.
	
	if ( parameters.size() < 2 )
	{
		ROS_ERROR("No parameters set (parameters.size = %d).", (int)parameters.size());
		result_.return_code = MISSING_PARAMETER_ERROR;
		sendResult( false );
		return;
	}

	std::string script_id   = parameters.at(0);
	std::string waypoint_id = parameters.at(1);

	ROS_DEBUG_NAMED(ROS_NAME, "Received script_id %s and waypoint_id %s", script_id.c_str(), waypoint_id.c_str());

	if (script_id == "place")
		determinePositionForPlace(waypoint, waypoint_id);
	else
		sendResult(false);
}	

void PositionDeterminator::executeItemAction( const std::string item_id, const std::vector<std::string> parameters )
{
	Item item = datamanager_->get<Item>(item_id);

	if (!item.get_bounding_box().isSet())
	{
		operator_gui_->action("Selecteer item");
		getParameter(item_id, PARAMETER_REQUEST::BOUNDING_BOX, boost::bind(&PositionDeterminator::continueItemAction, this, _1, _2));
	}
	else
		itemAction(item, parameters);
}

void PositionDeterminator::continueItemAction( const std::string item_id, const std::vector<std::string> parameters )
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
		itemAction(item, parameters);
}

void PositionDeterminator::itemAction( const Item item, const std::vector<std::string> parameters )
{
	ROS_DEBUG_NAMED(ROS_NAME, "PositionDeterminator::itemAction");
	//! @todo MdL: Check if they are available.
	
	if ( parameters.size() < 2 )
	{
		ROS_ERROR("No parameters set (parameters.size = %d).", (int)parameters.size());
		result_.return_code = MISSING_PARAMETER_ERROR;
		sendResult( false );
		return;
	}

	std::string script_id   = parameters.at(0);
	std::string waypoint_id = parameters.at(1);

	ROS_DEBUG_NAMED(ROS_NAME, "Received script_id %s and waypoint_id %s", script_id.c_str(), waypoint_id.c_str());

	if (script_id == "grab")
		determinePositionForGrab(item, waypoint_id);
	else
		sendResult(false);
}

bool PositionDeterminator::filteredOnHeight( const std::string frame, const double height, PointCloud& point_cloud )
{
	ROS_DEBUG_NAMED(ROS_NAME, "PositionDeterminator::filteredOnHeight");
	if ( not rose_transformations::transformToFrame(tf_, frame, point_cloud))
	{
		ROS_ERROR("Could not transform point cloud");
		return false;
	}

	vector<Point32> filtered_points;
	for ( auto& point : point_cloud.points)
		// Spaces between points in a surface are 0.5.
		if ( point.z > height - 0.026 && point.z < height + 0.026 )
			filtered_points.push_back(point);

	point_cloud.points = filtered_points;

	return true;
}

double PositionDeterminator::rotationAngle( const PoseStamped& pose, const Point32& target )
{
	//! @todo MdL: Make sure pose is in base_link.
	// rose_transformations::transformToFrame(tf_, "base_link", pose);
	return std::abs(rose_geometry::getShortestSignedAngle( target.x, target.y, tf::getYaw(pose.pose.orientation)));
}

bool PositionDeterminator::pickBestPose( const vector<PoseStamped>& poses, const PointCloud& targets, PoseStamped& best_pick )
{
	// Pick first target (for now)
	//! @todo MdL: Consider all target points.
	Point32 target = targets.points[0];

	ROS_DEBUG_NAMED(ROS_NAME, "PositionDeterminator::pickBestPose");
	
	PoseStamped base;
	base.header.frame_id = "base_link";
	base.pose.position.x = 0.0;
	base.pose.position.y = 0.0;
	base.pose.position.z = 0.0;
	base.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

	if ( poses.empty() ) 
	{
		ROS_ERROR("There are no feasable poses given");
		return false;
	}
	else 
		best_pick = poses.at(0);

	// Transform the base to the frame of the poses
	if ( not rose_transformations::transformToFrame(tf_, best_pick.header.frame_id, base) )
	{
		ROS_ERROR("Could not transform point");
		return false;
	}

	// Combine distance and rotation angle
	for ( auto& pose : poses )
		if ( rotationAngle(pose, target) <= rotationAngle(best_pick, target) && rose_geometry::distanceXYZ(pose, base) <= rose_geometry::distanceXYZ(best_pick, base) )
			best_pick = pose;

	ROS_DEBUG_NAMED(ROS_NAME, "Best pick: (%f, %f, %f)", best_pick.pose.position.x, best_pick.pose.position.y, best_pick.pose.position.z);
	
	publishPose(best_pick, false);

	return true;
}

bool PositionDeterminator::filterReachablePoints( PointCloud& point_cloud )
{
	reachable_points_before_filter_.publish(getMarkers(point_cloud, 5));
	// Filter out some points in the borders

	ROS_DEBUG_NAMED(ROS_NAME, "PositionDeterminator::filterReachablePoints");
	//! @todo MdL: Code can be improved.
	// Using two arrays to store points and their nr of points in the neighbourhood.
	// So, index X in array best_points references to index X in array nr_points_in_neighbourhood
	vector<Point32>	best_points;
	vector<int>     nr_points_in_neighbourhood;
	
	ROS_DEBUG_NAMED(ROS_NAME, "Received %d points", (int)point_cloud.points.size());

	int 	max_nr_poses = point_cloud.points.size() / 3;
	double 	max_dist 	 = 0.075; // max distance to neighbour

	//! @todo MdL: Fast implemented solution, can be optimized.
	// Filter out the points that lie outside of the point cloud.
	int min_value_stored = INT_MAX;

	for ( const auto& point : point_cloud.points )
	{
		int nr_poses_in_neighbourhood = 0;

		for ( const auto& pose_in_region : point_cloud.points )
			if ( rose_geometry::distanceXYZ(rose_conversions::point32ToPoint(point), rose_conversions::point32ToPoint(pose_in_region)) < max_dist )
				nr_poses_in_neighbourhood++;

		// The result poses is not yet full
		if ( best_points.size() < max_nr_poses )
		{
			// Keep track of the smallest value stored, this one is removed is a higher value is found
			min_value_stored = std::min(min_value_stored, nr_poses_in_neighbourhood);

			// Add point and nr of neighbours
			best_points.push_back(point);
			nr_points_in_neighbourhood.push_back(nr_poses_in_neighbourhood);
		}
		// Result values are full and value found is higher than lowest value stored
		else if ( nr_poses_in_neighbourhood > min_value_stored )
		{
			// Find the lowest value, keep track of the new lowest value
			int 	new_lowest_value = INT_MAX;
			int 	index_with_lowest_point;
			bool 	index_found = false;
			for ( int i = 0 ; i < best_points.size() ; i++ )
			{
				if ( nr_points_in_neighbourhood[i] == min_value_stored )
				{
					index_with_lowest_point = i;
					index_found = true;
				}
				else 
					new_lowest_value = std::min(new_lowest_value, nr_points_in_neighbourhood[i]);
			}

			// Remove the lowest value
			best_points.erase(best_points.begin() + index_with_lowest_point);
			nr_points_in_neighbourhood.erase( nr_points_in_neighbourhood.begin() + index_with_lowest_point);

			// Add new pose
			best_points.push_back(point);
			nr_points_in_neighbourhood.push_back(nr_poses_in_neighbourhood);

			// Update lowest value
			min_value_stored = new_lowest_value;
		}
		
	}

	// update the result variable
	point_cloud.points = best_points;
	reachable_points_after_filter_.publish(getMarkers(point_cloud, 15));

	return true;
}

bool PositionDeterminator::determinePositionForManipulation( PointCloud& target_points, PoseStamped& required_location )
{
	if ( not rose_transformations::transformToFrameNow(tf_, "base_link", target_points))
		ROS_ERROR("Could not transform points to base_link");

	// Get allowed positions
	PointCloud allowed_points = arm_controller_helper_->getReachablePointCloud();
	if ( not filteredOnHeight( target_points.header.frame_id, target_points.points[0].z, allowed_points ))
		ROS_ERROR("Could not get filtered points(1)");

	ROS_DEBUG_NAMED(ROS_NAME, "Found %d points near height %f", (int)allowed_points.points.size(), target_points.points[0].z);

	if ( not filterReachablePoints(allowed_points))
		ROS_ERROR("Could not get filtered points(2)");

	// Get new locations for the base
	rose_pose_explorer::reachable_poses reachability_msg;

	reachability_msg.request.target_points  = target_points;
	reachability_msg.request.allowed_points = allowed_points;

	if ( not reachable_position_service_.call(reachability_msg))
		ROS_ERROR("Problem getting reachable points");

	ROS_DEBUG_NAMED(ROS_NAME, "Received %d reachable points", (int)reachability_msg.response.reachable_points.size());

	if ( not pickBestPose(reachability_msg.response.reachable_points, target_points, required_location) )
	{
		ROS_ERROR("Could not find a best position");
		return false;
	}

	// Store the waypoint (in map frame)
	if ( not rose_transformations::transformToFrame(tf_, "base_link", required_location))
		ROS_ERROR("Could not transform waypoint to base_link");

	double current_x = -0.05;
	double current_y = 0.0;
	rose_geometry::rotateVect(&current_x, &current_y, tf::getYaw(required_location.pose.orientation));

	required_location.pose.position.x += current_x;
	required_location.pose.position.y += current_y;

	publishPose(required_location, true);

	return true;
}

void PositionDeterminator::determinePositionForPlace( Waypoint waypoint, const std::string waypoint_id )
{
	ROS_DEBUG_NAMED(ROS_NAME, "PositionDeterminator::determinePositionForPlace");

	PointCloud 	target_points;
	Point32     target_point;

	PoseStamped waypoint_pose = waypoint.getPoseStamped();

	target_point.x = waypoint_pose.pose.position.x;
	target_point.y = waypoint_pose.pose.position.y;
	target_point.z = waypoint_pose.pose.position.z;

	target_points.header.stamp    = ros::Time::now();
	target_points.header.frame_id = waypoint.getFrame();
	target_points.points.push_back(target_point);

	PoseStamped required_location;
	if ( not determinePositionForManipulation(target_points, required_location))
	{
		sendResult(false, "Kan er niet bij met de arm");
		return;
	}

	// Store waypoint
	Waypoint waypoint_to_store( waypoint_id, waypoint_id );
	waypoint_to_store.set(required_location);

	datamanager_->store<Waypoint>(waypoint_to_store);

	// Transform waypoint to map frame (because the robot is going drive after this operation)
	rose_transformations::transformToFrame(tf_, "map", waypoint_pose);
	waypoint.set(waypoint_pose);

	datamanager_->store<Waypoint>(waypoint);

	sendResult(true);
}

void PositionDeterminator::determinePositionForGrab( Item item, const std::string waypoint_id )
{
	ROS_DEBUG_NAMED(ROS_NAME, "PositionDeterminator::determinePositionForGrab");
	// Get target location
	BoundingBox bb 	= item.get_bounding_box();

	PointCloud 	target_points;
	Point32     target_point;

	target_point.x = bb.getCenter().x;
	target_point.y = bb.getCenter().y;
	target_point.z = bb.getCenter().z;

	target_points.header.stamp    = ros::Time::now();
	target_points.header.frame_id = bb.getFrame();
	target_points.points.push_back(target_point);

	PoseStamped required_location;
	determinePositionForManipulation(target_points, required_location);

	// Store waypoint
	Waypoint waypoint( waypoint_id, waypoint_id );
	waypoint.set(required_location);
	
	datamanager_->store<Waypoint>(waypoint);

	// Remove item bb from database
	BoundingBox empty_bb;
	item.set_bounding_box(empty_bb);

	datamanager_->store<Item>(item);

	sendResult(true);
}

void PositionDeterminator::publishPose( const PoseStamped pose, const bool corrected )
{
	visualization_msgs::Marker arrow;
    arrow.header 	= pose.header;
    arrow.ns        = "chosen_marker";
    arrow.pose		= pose.pose;
    
    arrow.action    = visualization_msgs::Marker::ADD;
    arrow.type      = visualization_msgs::Marker::ARROW;
    arrow.id        = 0;
    arrow.scale.x   = 0.20;
    arrow.scale.y   = 0.04;
    arrow.scale.z   = 0.04;
    arrow.color.b   = 1.0f;
    arrow.color.a   = 1.0f;

    arrow.lifetime  = ros::Duration(100);

    if (corrected)
    {
    	arrow.color.g   = 1.0f;
    	best_pose_corrected_pub_.publish(arrow);
    }
    else
    {
    	arrow.color.b   = 1.0f;
		best_pose_pub_.publish(arrow);
    }
}

MarkerArray  PositionDeterminator::getMarkers( const PointCloud point_cloud, int lifetime )
{
	MarkerArray markers;
   	int i = 0;

    for ( auto& point : point_cloud.points )
    {
		visualization_msgs::Marker marker;
	    marker.header 	 = point_cloud.header;
	    marker.ns        = "markers" + point_cloud.header.seq;

	    marker.pose.position.x    = point.x;
	    marker.pose.position.y    = point.y;
	    marker.pose.position.z    = point.z;
	    marker.pose.orientation.w = 1.0f;
	    
	    marker.action    = visualization_msgs::Marker::ADD;
	    marker.type      = visualization_msgs::Marker::SPHERE;
	    marker.id        = i++;
	    marker.scale.x   = 0.03;
	    marker.scale.y   = 0.03;
	    marker.scale.z   = 0.03;
	    marker.color.g   = 1.0f;
	    marker.color.a   = 1.0f;

	    marker.lifetime  = ros::Duration(lifetime);

		markers.markers.push_back(marker);
    }

    ROS_DEBUG_NAMED(ROS_NAME, "Publishing %d markers", (int)markers.markers.size() );

    return markers;
}