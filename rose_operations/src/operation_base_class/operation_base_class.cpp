/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
* Author: Mathijs de Langen
* Date  : 2014/02/14
*     - File created.
*
* Description:
* description
* 
***********************************************************************************/
#include "operation_base_class/operation_base_class.hpp"

OperationBaseClass::OperationBaseClass( std::string name, ros::NodeHandle n )
	: name_ ( name )
	, n_ ( n )
{
	smc_ 					= new SMC(n_, name_, boost::bind(&OperationBaseClass::CB_goalReceived, this, _1, _2),
							  					  boost::bind(&OperationBaseClass::CB_serverCancel, this, _1) );

	operator_gui_ 			= new OperatorMessaging(n_);
		
	datamanager_ 			= new DatamanagerAPI();

	addArmClients();

	// Paramater manager client
	addParameterManagerClient();

	// Visual servoing client
	addArmVisualServoingClient();

	// Standard result
	result_.return_code = NODE_NOT_IMPLEMENTED_ERROR;
}

OperationBaseClass::~OperationBaseClass()
{

}

void OperationBaseClass::startOperation()
{
	smc_->startServer();
}

void OperationBaseClass::addParameterManagerClient()
{
	// Add parameter client
	smc_->addClient<ParameterAction>("parameter_manager", 
	   	boost::bind(&OperationBaseClass::CB_getParameterSuccess, this, _1, _2),
	   	boost::bind(&OperationBaseClass::CB_getParameterFail, this, _1, _2),
	   	boost::bind(&OperationBaseClass::CB_getParameterActive, this),
	   	boost::bind(&OperationBaseClass::CB_getParameterFeedback, this, _1));
}

void OperationBaseClass::addArmVisualServoingClient()
{
	smc_->addClient<ArmVisualServoAction>("arm_visual_servoing",
		boost::bind(&OperationBaseClass::CB_armVisualServoingSuccess, this, _1, _2),
	    boost::bind(&OperationBaseClass::CB_armVisualServoingFail, this, _1, _2),
	   	boost::bind(&OperationBaseClass::CB_armVisualServoingActive, this),
	   	boost::bind(&OperationBaseClass::CB_armVisualServoingFeedback, this, _1));
}

// Client action
void OperationBaseClass::getParameter( std::string item_id, PARAMETER_REQUEST parameter, ParameterSucces parameter_succes)
{
	rose_parameter_manager::parameterGoal goal;
	goal.item_id 		= item_id;
	goal.parameter 		= parameter;

    ROS_DEBUG("OperationBaseClass::getParameter: item_id: %s parameter: %s", item_id.c_str(), PARAMETER_REQUEST_STRINGS[parameter]);
	
	smc_->sendGoal<ParameterAction>(goal, "parameter_manager");

	parameter_succes_ 	= parameter_succes;
}

void OperationBaseClass::CB_getParameterSuccess( const actionlib::SimpleClientGoalState& state, const rose_parameter_manager::parameterResultConstPtr& result )
{
    ROS_INFO("OperationBaseClass::CB_getParametersuccess");

    parameter_succes_(current_item_id_, current_parameters_ids_ );
}

void OperationBaseClass::CB_getParameterFail( const actionlib::SimpleClientGoalState& state, const rose_parameter_manager::parameterResultConstPtr& result )
{
  	ROS_INFO("OperationBaseClass::CB_getParameterfail");

  	result_.return_code = MISSING_PARAMETER_ERROR;
  	sendResult( false );
}

void OperationBaseClass::CB_getParameterActive()
{
	ROS_INFO("OperationBaseClass::CB_getParameteractive");
}

void OperationBaseClass::CB_getParameterFeedback( const rose_parameter_manager::parameterFeedbackConstPtr& feedback )
{
	ROS_INFO("OperationBaseClass::CB_getParameterfeedback");
}

void OperationBaseClass::sendResult( bool succes )
{
	if ( succes )
	{
		result_.return_code = SUCCESS;
		operator_gui_->message("Operatie geslaagd"/*toupper(name_.c_str()[0])+" finished succesfully"*/);
	}
	else
	{
		// operator_gui_->message("Received an error"/*toupper(name_.c_str()[0])+"%s had an error"*/);
	}
	smc_->sendServerResult( succes, result_ );
}

void OperationBaseClass::sendResult( bool succes, std::string message )
{
    if ( succes )
    {
        result_.return_code = SUCCESS;
        operator_gui_->message(message);
    }
    else
    {
        operator_gui_->warn(message);
    }
    smc_->sendServerResult( succes, result_ );
}

void OperationBaseClass::CB_goalReceived( const rose_operations::basic_operationGoalConstPtr& goal, SMC* smc )
{
    current_parameters_ids_ = goal->parameter_ids;

    if(goal->item_ids.size() > 1) //goal->item_ids is a list of roscomm::stringlist.
    {
        current_item_ids_ = goal->item_ids;
        executeMultiItemAction(current_item_ids_, current_parameters_ids_);
    }
    else
        current_item_id_ 		= goal->item_ids.at(0);

	if ( boost::starts_with(current_item_id_, Item::IDENTIFIER) )
		executeItemAction(current_item_id_, current_parameters_ids_);

	else if ( boost::starts_with(current_item_id_, Waypoint::IDENTIFIER) )
		executeWaypointAction(current_item_id_, current_parameters_ids_);

	else if ( boost::starts_with(current_item_id_, Person::IDENTIFIER) )
		executePersonAction(current_item_id_, current_parameters_ids_);

	else
	{
        ROS_ERROR("Cannot identify object with id %s", current_item_id_.c_str());
		sendResult(false);
	}
}

void OperationBaseClass::CB_serverCancel( SMC* smc )
{
	ROS_INFO("OperationBaseClass::CB_serverCancel");
	operator_gui_->message("Operation cancelled");
}

void OperationBaseClass::executeItemAction( const std::string item_id, const std::vector<std::string> parameters )
{
	ROS_ERROR("OperationBaseClass, function executeItemAction() not implemented.");
}

void OperationBaseClass::executeMultiItemAction( const std::vector<std::string> item_ids, const std::vector<std::string> parameters )
{
    ROS_ERROR("OperationBaseClass, function executeMultiItemAction() not implemented.");
}

void OperationBaseClass::executeWaypointAction( const std::string item_id, const std::vector<std::string> parameters )
{
	ROS_ERROR("OperationBaseClass, function executeWaypointAction() not implemented.");
}

void OperationBaseClass::executePersonAction( const std::string item_id, const std::vector<std::string> parameters )
{
	ROS_ERROR("OperationBaseClass, function executePersonAction() not implemented.");
}
