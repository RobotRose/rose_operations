/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/02/14
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef MOVE_TO_HPP
#define MOVE_TO_HPP

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>

#include <tf/transform_listener.h>

#include "operation_base_class/operation_base_class.hpp"

#include "rose_common/common.hpp"

class MoveTo : public OperationBaseClass
{
  public:
	MoveTo( std::string name, ros::NodeHandle n );
	~MoveTo();

  private:
  	void CB_moveSuccess( const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  	void CB_moveFail( const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);

    bool checkWaypoint( const Waypoint waypoint );

    void executeWaypointAction( const std::string item_id, const std::vector<std::string> parameters = std::vector<std::string>() );
    void continueWaypointAction( const std::string item_id, const std::vector<std::string> parameters = std::vector<std::string>()  );

  	void executeItemAction( const std::string item_id, const std::vector<std::string> parameters = std::vector<std::string>()  );
    void continueItemAction( const std::string item_id, const std::vector<std::string> parameters = std::vector<std::string>()  );

    void moveToPointNearItem ( Item item );
  	void moveToLocation( const Waypoint waypoint );

  	ros::Publisher           location_publisher_;
    tf::TransformListener    tf_;
};

#endif //MOVE_TO_HPP