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
#include "rose_operation_manager/operation_manager.hpp"

OperationManager::OperationManager( std::string name, ros::NodeHandle n )
	: name_ ( name )
	, n_ ( n )
	, running_operations_ ( false )
	, nr_fails_ ( 0 )
{
	smc_ = new SMC(n_, name_, boost::bind(&OperationManager::CB_serverWork, this, _1, _2),
						      boost::bind(&OperationManager::CB_serverCancel, this, _1));
	// add all clients operations
	//! @todo: Get all (basic) scripts from the datamanager
	clients_.push_back("grab");
	clients_.push_back("move_to");
	clients_.push_back("say");
	clients_.push_back("place");
	clients_.push_back("give");	
	clients_.push_back("handover_to_rose");	
	clients_.push_back("arm_driving_position");	
	clients_.push_back("arm_grabbing_position");	
	clients_.push_back("open_gripper");	
	clients_.push_back("close_gripper");	
	clients_.push_back("strech");	
	clients_.push_back("position_determinator");	
	addClients();

	smc_->startServer();

	action_planner_service_client_ = n_.serviceClient<rose_action_planner::recover>("/action_planner/recover");
}

OperationManager::~OperationManager()
{

}

void OperationManager::addClients()
{
	std::string client;
	BOOST_FOREACH(client, clients_)
	{
		ROS_INFO("add client %s", client.c_str());
		smc_->addClient<ActionMessage>("/basic_operation/" + client, 
			boost::bind(&OperationManager::CB_action_success, this, _1, _2),
			boost::bind(&OperationManager::CB_action_fail, this, _1, _2),
			NULL, NULL);
		ROS_INFO("add client %s DONE", client.c_str());
	}
}

// Client callbacks
void OperationManager::CB_action_success( const actionlib::SimpleClientGoalState& state, const rose_operations::basic_operationResultConstPtr& result )
{
	ROS_INFO_NAMED(ROS_NAME, "OperationManager::CB_action_success");
	// Last action was succesful
	if ( operation_list_.empty())
		sendResult(true, ACTION_RESULT::SUCCESS);
	else
		runNextOperation();
}

void OperationManager::CB_action_fail( const actionlib::SimpleClientGoalState& state, const rose_operations::basic_operationResultConstPtr& result )
{
	nr_fails_++;
	if ( nr_fails_ > MAX_NR_FAILS )
	{
		ROS_INFO_NAMED(ROS_NAME, "Failed too many times, sending result (unsuccessful)");
		operation_list_.clear();
		sendResult(false, static_cast<ACTION_RESULT>(result->return_code));
		return;
	}

	//! @todo MdL: Fix M_range_check somewhere where, when action planner does not return a list of scripts?.

	ROS_DEBUG_NAMED(ROS_NAME, "OperationManager::CB_action_fail");

	ROS_INFO_NAMED(ROS_NAME, "received return code: %d", result->return_code);

	// Talk to the action planner
	rose_action_planner::recover   planning_request;
    planning_request.request.current_script = smc_->getLastGoal()->script_id;
    for ( auto& item_id : smc_->getLastGoal()->item_ids.at(0).values )
		planning_request.request.item_ids.push_back(item_id);

    if ( smc_->getLastGoal()->item_ids.size() > 1 )
		for ( auto& param_id : smc_->getLastGoal()->item_ids.at(1).values )
			planning_request.request.param_ids.push_back(param_id);

    planning_request.request.return_code 	= result->return_code;

	action_planner_service_client_.call(planning_request);
	if ( not planning_request.response.recover_scripts.empty() )
	{
		addPlanningToOperationList(planning_request);
		runNextOperation();
	}
	else
		sendResult(false, static_cast<ACTION_RESULT>(result->return_code));
}

void OperationManager::addPlanningToOperationList( rose_action_planner::recover planning_request )
{
	ROS_DEBUG_NAMED(ROS_NAME, "OperationManager::addPlanningToOperationList");
	tuple<rose_operations::basic_operationGoal, std::string> operation;

	ROS_DEBUG_NAMED(ROS_NAME, "%d recover scripts received", (int)planning_request.response.recover_scripts.size());
	// Move to the list backwards
	//! @todo MdL: Remove i, use iterator(?)
	for ( int i = planning_request.response.recover_scripts.size() ; i > 0 ; i-- )
	{
		operation = createOperation(planning_request.response.recover_item_ids.at(i-1), planning_request.response.recover_param_ids.at(i-1), planning_request.response.recover_scripts.at(i-1));
		operation_list_.push_front(operation);
	}
}

// Server callbacks
void OperationManager::sendResult( bool succes, ACTION_RESULT result_code )
{
	ROS_INFO("OperationManager::sendResult::begin");
	
	rose_operation_manager::executeResult result;
	result.return_code = result_code;
	smc_->sendServerResult( succes, result );

	ROS_INFO("OperationManager::sendResult::end");
}

void OperationManager::CB_serverWork( const rose_operation_manager::executeGoalConstPtr& goal, SMC* smc )
{
	nr_fails_ = 0;

	ROS_INFO("OperationManager::CB_serverWork::begin");
	if ( goal->item_ids.size() == 0) //! @todo MdL: Figure out what to do if no item_ids (e.g. strech).
	{
		roscomm::stringlist stringlist;
		roscomm::stringlist param_ids;
		stringlist.values.push_back("dummy");

        tuple<rose_operations::basic_operationGoal, std::string> operation = createOperation(stringlist, param_ids, goal->script_id);
		operation_list_.push_back(operation);
		runNextOperation();
	}
	else if ( goal->item_ids.size() > 0 )
	{
        ROS_DEBUG_NAMED(ROS_NAME, "Received call to '%s' with %i stringlists in item_ids",
                        goal->script_id.c_str(), (int)goal->item_ids.size());

        //! @todo LvB: What to do with 1+ items specified, e.g. for the place-operation?
        // Only first item
        roscomm::stringlist item_ids = goal->item_ids[0];
        roscomm::stringlist param_ids;

        if ( goal->item_ids.size() > 1 )
        	for ( int i = 1 ; i < goal->item_ids.size() ; i++ )
            	param_ids.values.insert(param_ids.values.end(), goal->item_ids[i].values.begin(), goal->item_ids[i].values.end() );

        ROS_DEBUG_NAMED(ROS_NAME, "Creating and queueing a %s operation with %i item_ids and %i parameters:",
                        goal->script_id.c_str(), (int)item_ids.values.size(), (int)param_ids.values.size());

        tuple<rose_operations::basic_operationGoal, std::string> operation = createOperation(item_ids, param_ids, goal->script_id);

		operation_list_.push_back(operation);
		runNextOperation();
	}

	ROS_INFO("OperationManager::CB_serverWork::end");
}

tuple<rose_operations::basic_operationGoal, std::string> OperationManager::createOperation( const roscomm::stringlist item_ids, const roscomm::stringlist parameter_ids, const std::string script_id )
{
	//! @todo MdL: Support multiple items
    // Check if script has item ids
	rose_operations::basic_operationGoal client_goal;

	for ( auto& item_id : item_ids.values )
		client_goal.item_ids.push_back(item_id);

	for ( auto& param_id : parameter_ids.values )
		client_goal.parameter_ids.push_back(param_id);

	return tuple<rose_operations::basic_operationGoal, std::string>(client_goal, "/basic_operation/" + script_id);
}

void OperationManager::runNextOperation()
{
	ROS_DEBUG_NAMED(ROS_NAME, "Current list of scripts:");
	for ( auto& operation_tuple : operation_list_ )
		ROS_DEBUG_NAMED(ROS_NAME, "%s", std::get<1>(operation_tuple).c_str());

	if ( not operation_list_.empty())
	{
		tuple<rose_operations::basic_operationGoal, std::string> operation = operation_list_.front();
		operation_list_.pop_front();

		smc_->sendGoal<ActionMessage>(std::get<0>(operation), std::get<1>(operation));
	}
}	

void OperationManager::CB_serverCancel( SMC* smc )
{
	ROS_DEBUG_NAMED(ROS_NAME, "OperationManager::Cancel received");
	operation_list_.clear();
	ROS_DEBUG_NAMED(ROS_NAME, "List of pending operations has been cleared.");
}
