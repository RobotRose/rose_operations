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

#include "rose_datamanager_api/datamanager_api.hpp"

#include "rose_arm_controller_msgs/move_to_tfAction.h"
#include "rose_arm_controller_msgs/move_to_tfGoal.h"
#include "rose_arm_controller_msgs/move_to_tfResult.h"
#include "rose_arm_controller_msgs/move_to_tfFeedback.h"

#include "rose_arm_controller_msgs/set_positionAction.h"
#include "rose_arm_controller_msgs/set_positionGoal.h"
#include "rose_arm_controller_msgs/set_positionResult.h"
#include "rose_arm_controller_msgs/set_positionFeedback.h"

#include "rose_arm_controller_msgs/set_gripper_widthAction.h"
#include "rose_arm_controller_msgs/set_gripper_widthGoal.h"
#include "rose_arm_controller_msgs/set_gripper_widthResult.h"
#include "rose_arm_controller_msgs/set_gripper_widthFeedback.h"

#include "rose_arm_controller_msgs/set_velocityAction.h"
#include "rose_arm_controller_msgs/set_velocityGoal.h"
#include "rose_arm_controller_msgs/set_velocityResult.h"
#include "rose_arm_controller_msgs/set_velocityFeedback.h"

#include "rose_arm_controller_msgs/set_wrenchAction.h"
#include "rose_arm_controller_msgs/set_wrenchGoal.h"
#include "rose_arm_controller_msgs/set_wrenchResult.h"
#include "rose_arm_controller_msgs/set_wrenchFeedback.h"

#include "rose_operations/basic_operationAction.h"
#include "rose_operations/basic_operationGoal.h"
#include "rose_operations/basic_operationResult.h"
#include "rose_operations/basic_operationFeedback.h"

#include "rose_parameter_manager/parameterAction.h"
#include "rose_parameter_manager/parameterGoal.h"
#include "rose_parameter_manager/parameterFeedback.h"
#include "rose_parameter_manager/parameterResult.h"
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
  	typedef rose_parameter_manager::parameterAction ParameterAction;
  	typedef rose_arm_controller_msgs::move_to_tfAction ArmVisualServoAction;
  	typedef boost::function< void (const std::string item_id, const std::vector<std::string> parameter_ids) > ParameterSucces;

	OperationBaseClass( std::string name, ros::NodeHandle n );
	~OperationBaseClass();

  protected:
    typedef ServerMultipleClient<rose_operations::basic_operationAction> SMC;

	virtual void CB_serverCancel( SMC* smc );
	virtual void CB_goalReceived( const rose_operations::basic_operationGoalConstPtr& goal, SMC* smc );
	
	virtual void startOperation();
	virtual void sendResult( bool succes );
    virtual void sendResult( bool succes,  std::string message );

	virtual void getParameter( std::string item_id, PARAMETER_REQUEST parameter, ParameterSucces parameter_succes);
	virtual void CB_getParameterSuccess( const actionlib::SimpleClientGoalState& state, const rose_parameter_manager::parameterResultConstPtr& result );
	virtual void CB_getParameterFail( const actionlib::SimpleClientGoalState& state, const rose_parameter_manager::parameterResultConstPtr& result );
	virtual void CB_getParameterActive();
	virtual void CB_getParameterFeedback( const rose_parameter_manager::parameterFeedbackConstPtr& feedback );
ad
	virtual void executeItemAction( const std::string id, const std::vector<std::string> parameters = std::vector<std::string>() );
    virtual void executeMultiItemAction( const std::vector<std::string> item_ids, const std::vector<std::string> parameters = std::vector<std::string>() );
    virtual void executePersonAction( const std::string id, const std::vector<std::string> parameters = std::vector<std::string>() );
    virtual void executeWaypointAction( const std::string id, const std::vector<std::string> parameters = std::vector<std::string>() );

    void addParameterManagerClient();
	void addArmVisualServoingClient();

	std::string 						name_;
	ros::NodeHandle 					n_;

	ArmControllerHelper*				arm_controller_helper_;
	DatamanagerAPI*						datamanager_;
	SMC*								smc_;
    OperatorMessaging*                  operator_gui_;

	ParameterSucces 					parameter_succes_;
	
	rose_operations::basic_operationResult 	result_;
	std::string 							current_item_id_;
    std::vector<std::string>				current_item_ids_;
	std::vector<std::string>				current_parameters_ids_;
	
	tf::TransformListener 				tf_;
};

#endif //OPERATION_BASE_CLASS_HPP
