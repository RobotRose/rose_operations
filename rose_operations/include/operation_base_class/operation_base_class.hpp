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
#ifndef OPERATION_BASE_CLASS_HPP
#define OPERATION_BASE_CLASS_HPP

#include <boost/algorithm/string/predicate.hpp>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_listener.h>

#include "action_result_message.hpp"

#include "arm_controller_helper.hpp"

#include "rose_datamanager_api/datamanager_api.hpp"

#include "arm_controller/move_to_tfAction.h"
#include "arm_controller/move_to_tfGoal.h"
#include "arm_controller/move_to_tfResult.h"
#include "arm_controller/move_to_tfFeedback.h"

#include "operations/basic_operationAction.h"
#include "operations/basic_operationGoal.h"
#include "operations/basic_operationResult.h"
#include "operations/basic_operationFeedback.h"

#include "parameter_manager/parameterAction.h"
#include "parameter_manager/parameterGoal.h"
#include "parameter_manager/parameterFeedback.h"
#include "parameter_manager/parameterResult.h"
#include "parameter_request_message.hpp"

#include "server_multiple_client/server_multiple_client.hpp"
#include "operator_messaging/operator_messaging.hpp"
#include "std_msgs/String.h"
#include "roscomm/stringlist.h"

#include "item/item.hpp"
#include "person/person.hpp"
#include "waypoint/waypoint.hpp"
#include "resource/resource.hpp"

class OperationBaseClass
{
  public:
  	typedef parameter_manager::parameterAction ParameterAction;
  	typedef arm_controller::move_to_tfAction ArmVisualServoAction;
  	typedef boost::function< void (const std::string item_id, const std::vector<std::string> parameter_ids) > ParameterSucces;

	OperationBaseClass( std::string name, ros::NodeHandle n );
	~OperationBaseClass();

  protected:
    typedef ServerMultipleClient<operations::basic_operationAction> SMC;

	virtual void CB_serverCancel( SMC* smc );
	virtual void CB_goalReceived( const rose_operations::basic_operationGoalConstPtr& goal, SMC* smc );
	
	virtual void startOperation();
	virtual void sendResult( bool succes );
    virtual void sendResult( bool succes,  std::string message );

	virtual void getParameter( std::string item_id, PARAMETER_REQUEST parameter, ParameterSucces parameter_succes);
	virtual void CB_getParameterSuccess( const actionlib::SimpleClientGoalState& state, const parameter_manager::parameterResultConstPtr& result );
	virtual void CB_getParameterFail( const actionlib::SimpleClientGoalState& state, const parameter_manager::parameterResultConstPtr& result );
	virtual void CB_getParameterActive();
	virtual void CB_getParameterFeedback( const parameter_manager::parameterFeedbackConstPtr& feedback );

	virtual void CB_armVisualServoingSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::move_to_tfResultConstPtr& result );
	virtual void CB_armVisualServoingFail( const actionlib::SimpleClientGoalState& state, const arm_controller::move_to_tfResultConstPtr& result );
	virtual void CB_armVisualServoingActive();
	virtual void CB_armVisualServoingFeedback( const arm_controller::move_to_tfFeedbackConstPtr& feedback );

	virtual void CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );
	virtual void CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );
	virtual void CB_armActionActive();
	virtual void CB_armActionFeedback( const arm_controller::manipulateFeedbackConstPtr& feedback );

	virtual void executeItemAction( const std::string id, const std::vector<std::string> parameters = std::vector<std::string>() );
    virtual void executeMultiItemAction( const std::vector<std::string> item_ids, const std::vector<std::string> parameters = std::vector<std::string>() );
    virtual void executePersonAction( const std::string id, const std::vector<std::string> parameters = std::vector<std::string>() );
    virtual void executeWaypointAction( const std::string id, const std::vector<std::string> parameters = std::vector<std::string>() );

    void addArmClients();
    void addParameterManagerClient();
	void addArmVisualServoingClient();

	std::string 						name_;
	ros::NodeHandle 					n_;

	ArmControllerHelper*				arm_controller_helper_;
	DatamanagerAPI*						datamanager_;
	SMC*								smc_;
    OperatorMessaging*                  operator_gui_;

	ParameterSucces 					parameter_succes_;
	
	operations::basic_operationResult 	result_;
	std::string 						current_item_id_;
    std::vector<std::string>			current_item_ids_;
	std::vector<std::string>			current_parameters_ids_;
	
	tf::TransformListener 				tf_;
};

#endif //OPERATION_BASE_CLASS_HPP
