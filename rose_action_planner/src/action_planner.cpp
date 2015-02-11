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
#include "rose_action_planner/action_planner.hpp"

ActionPlanner::ActionPlanner( std::string name, ros::NodeHandle n )
	: n_ ( n )
	, name_ ( name )
{
	action_planner_service_ 	= n_.advertiseService( name_ + "/recover", &ActionPlanner::CB_planRecover, this );
}

ActionPlanner::~ActionPlanner()
{
	
}

bool ActionPlanner::CB_planRecover( action_planner::recover::Request  &req,	
									action_planner::recover::Response &res)
{
	switch ( req.return_code )
	{
		case ACTION_RESULT::OUT_OF_REACH_ERROR:
			recoverGrabFromOutOfReach(req, res);
			break;
	}

	return true;
}


void ActionPlanner::recoverGrabFromOutOfReach( action_planner::recover::Request  &req,
											   action_planner::recover::Response &res )
{
	if ( req.item_ids.size() != 1 )
	 	ROS_ERROR("To many item ids (>1)");
	
	vector<std::string> item_ids;
	vector<std::string> param_ids;
	roscomm::stringlist item_stringlist;
	roscomm::stringlist param_stringlist;

	// Calculate position
	res.recover_scripts.push_back("position_determinator");
	item_ids.push_back(req.item_ids.at(0));
	param_ids.push_back("grab");
	param_ids.push_back("waypoint10");
	item_stringlist.values = item_ids;
	param_stringlist.values = param_ids;
	res.recover_item_ids.push_back(item_stringlist);
	res.recover_param_ids.push_back(param_stringlist);

	// Move_to position
	res.recover_scripts.push_back("move_to");
	item_ids.clear();
	param_ids.clear();
	item_ids.push_back("waypoint10");
	item_stringlist.values = item_ids;
	param_stringlist.values = param_ids;
	res.recover_item_ids.push_back(item_stringlist);
	res.recover_param_ids.push_back(param_stringlist);

	res.recover_scripts.push_back("grab");
	item_ids.clear();
	param_ids.clear();
	item_ids.push_back(req.item_ids.at(0));
	item_stringlist.values = item_ids;
	param_stringlist.values = req.param_ids;
	res.recover_item_ids.push_back(item_stringlist);
	res.recover_param_ids.push_back(param_stringlist);
}