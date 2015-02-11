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
#ifndef OPERATION_MANAGER_HPP
#define OPERATION_MANAGER_HPP

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>


#include "operations/basic_operationAction.h"
#include "operations/basic_operationGoal.h"
#include "operations/basic_operationResult.h"
#include "operations/basic_operationFeedback.h"

#include "rose_operation_manager/executeAction.h"
#include "rose_operation_manager/executeGoal.h"
#include "rose_operation_manager/executeResult.h"
#include "rose_operation_manager/executeFeedback.h"

#include "rose_action_planner/recover.h"

#include "std_msgs/String.h"

#include "action_result_message.hpp"
#include "luctor_base_class/luctor_base_class.hpp"
#include "operation_base_class/operation_base_class.hpp"
#include "server_multiple_client/server_multiple_client.hpp"

#define MAX_NR_FAILS 0 // Allowed nr of fails. 0 means no mitigation of operations. Changing this now will (possibly) crash the OM

using std::list;
using std::tuple;
using std::vector;

class OperationManager
{
    typedef rose_operation_manager::executeAction         OperationManagerMessage;
    typedef operations::basic_operationAction             ActionMessage;
    typedef ServerMultipleClient<OperationManagerMessage> SMC;

  public:
	OperationManager( std::string name, ros::NodeHandle n );
	~OperationManager();

  private:
    void addClients();

    void CB_serverCancel( SMC* smc );
    void CB_serverWork( const rose_operation_manager::executeGoalConstPtr& goal, SMC* smc );
    void sendResult(bool succes, ACTION_RESULT result_code);

    // Grab
    void CB_action_success( const actionlib::SimpleClientGoalState& state, const operations::basic_operationResultConstPtr& result );
    void CB_action_fail( const actionlib::SimpleClientGoalState& state, const operations::basic_operationResultConstPtr& result );

    void runNextOperation();

    void addPlanningToOperationList( rose_action_planner::recover planning_request );
    tuple<operations::basic_operationGoal, std::string> createOperation( const roscomm::stringlist item_ids, const roscomm::stringlist parameter_ids, const std::string script_id );

    bool                                    running_operations_;
    int                                     nr_fails_;
  	std::string 		                    name_;
  	ros::NodeHandle                         n_;
    ros::ServiceClient                      action_planner_service_client_;

  	SMC*	                                smc_;
    vector<std::string>	                    clients_;
    list<tuple<operations::basic_operationGoal, std::string>> operation_list_;
};

#endif //OPERATION_MANAGER_HPP
