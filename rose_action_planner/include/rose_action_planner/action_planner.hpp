/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/07/22
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef ACTION_PLANNER_HPP
#define ACTION_PLANNER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include "rose_action_planner/recover.h"

#include "action_result_message.hpp"
#include "arm_controller_helper.hpp"
#include "rose_datamanager_api/datamanager_api.hpp"

using geometry_msgs::PointStamped; 
using std::vector;

class ActionPlanner
{
  public:
    ActionPlanner( std::string name, ros::NodeHandle n );
    ~ActionPlanner();

  private:
  	bool CB_planRecover( action_planner::recover::Request  &req,	
						 action_planner::recover::Response &res );

    void recoverGrabFromOutOfReach( action_planner::recover::Request  &req,
                                    action_planner::recover::Response &res );

  	std::string 	        name_;
  	ros::NodeHandle         n_;

    ros::ServiceServer      action_planner_service_;
};

#endif //ACTION_PLANNER_HPP