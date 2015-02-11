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
#ifndef PARAMETER_MANAGER_HPP
#define PARAMETER_MANAGER_HPP

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

#include "rose_parameter_manager/parameterAction.h"
#include "rose_parameter_manager/parameterGoal.h"
#include "rose_parameter_manager/parameterFeedback.h"
#include "rose_parameter_manager/parameterResult.h"

#include "parameter_request_message.hpp"

#include "server_multiple_client/server_multiple_client.hpp"

class ParameterManager
{
  	typedef rose_parameter_manager::parameterAction ParameterAction;
  	typedef ServerMultipleClient<ParameterAction> SMC;

  public:
	ParameterManager( std::string name, ros::NodeHandle n );
	~ParameterManager();

  private:
  	void addClients();

	void CB_serverCancel( SMC* smc );
	void CB_serverWork( const rose_parameter_manager::parameterGoalConstPtr& goal, SMC* smc );
	void sendResult(bool succes);

  // Custom client succes/fail
	void CB_action_success( const actionlib::SimpleClientGoalState& state, const rose_parameter_manager::parameterResultConstPtr& result );
	void CB_action_fail( const actionlib::SimpleClientGoalState& state, const rose_parameter_manager::parameterResultConstPtr& result );

  	std::string 			   name_;
  	ros::NodeHandle 		   n_;
  	SMC*		               smc_;
    std::vector<std::string>   clients_;
};

#endif //PARAMETER_MANAGER_HPP