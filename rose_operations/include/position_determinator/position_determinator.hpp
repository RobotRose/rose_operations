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
*   an item. It stores this pose as a waypoint in the database to be used.
*   
***********************************************************************************/
#ifndef POSITION_DETERMINATOR_HPP
#define POSITION_DETERMINATOR_HPP

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "rose_pose_explorer/reachable_poses.h"

#include "operation_base_class/operation_base_class.hpp"
#include "rose_common/common.hpp"
#include "rose_transformations/transformations.hpp"

using geometry_msgs::Point32; 
using geometry_msgs::PointStamped; 
using geometry_msgs::PoseStamped; 
using rose_geometry::distanceXYZ;
using sensor_msgs::PointCloud;
using std::vector;
using std::map;
using visualization_msgs::MarkerArray;

class PositionDeterminator : public OperationBaseClass
{
  public:
    PositionDeterminator( std::string name, ros::NodeHandle n );
    ~PositionDeterminator();

  private:
    void executeItemAction( const std::string item_id, const std::vector<std::string> parameters = std::vector<std::string>()  );
    void continueItemAction( const std::string item_id, const std::vector<std::string> parameters = std::vector<std::string>()  );

    void executeWaypointAction( const std::string item_id, const std::vector<std::string> parameters );
    void continueWaypointAction( const std::string item_id, const std::vector<std::string> parameters );

    void itemAction( const Item item, const std::vector<std::string> parameters );
    void waypointAction( const Waypoint waypoint, const std::vector<std::string> parameters );

    void determinePositionForGrab( Item item, const std::string location_id );
    void determinePositionForPlace( Waypoint waypoint, const std::string waypoint_id );
    bool determinePositionForManipulation( PointCloud& target_points, PoseStamped& required_location );

    double      rotationAngle( const PoseStamped& pose, const Point32& target );
    bool        pickBestPose( const vector<PoseStamped>& points, const PointCloud& targets, PoseStamped& best_pose );
    bool        filteredOnHeight( const std::string frame, const double height, PointCloud& points );
    bool        filterReachablePoints( PointCloud& poses );

    void        publishPose( const PoseStamped pose, const bool corrected );
    void        publishReachablePoints( const PointCloud points );
    MarkerArray getMarkers( const PointCloud point_cloud, int lifetime );

    tf::TransformListener   tf_;
    ros::Publisher          best_pose_pub_;
    ros::Publisher          best_pose_corrected_pub_;
    ros::Publisher          reachable_points_after_filter_;
    ros::Publisher          reachable_points_before_filter_;
    ros::ServiceClient      reachable_position_service_;
};

#endif //POSITION_DETERMINATOR_HPP