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
#include "rose_parameter_manager/	parameter_manager.hpp"

ParameterManager::ParameterManager( std::string name, ros::NodeHandle n )
	: name_ ( name )
	, n_ ( n )
{
	ROS_INFO("ParameterManager::ParameterManager()::begin");

	smc_ = new SMC(n_, name_, boost::bind(&ParameterManager::CB_serverWork, this, _1, _2),
                       boost::bind(&ParameterManager::CB_serverCancel, this, _1));

	// add all clients
	clients_.push_back("/text_selector");
	clients_.push_back("/overview_camera");
	clients_.push_back("/map_display");
	addClients();

	smc_->startServer();

	ROS_INFO("ParameterManager::ParameterManager()::end");
}

ParameterManager::~ParameterManager()
{

}

void ParameterManager::addClients()
{
	ROS_INFO("ParameterManager::addClients()::begin");

	std::string client;
	BOOST_FOREACH(client, clients_)
	{
		smc_->addClient<ParameterAction>(client, 
			boost::bind(&ParameterManager::CB_action_success, this, _1, _2),
			boost::bind(&ParameterManager::CB_action_fail, this, _1, _2),
			NULL, NULL);
	}

	ROS_INFO("ParameterManager::addClients()::end");
}

// Client callbacks
void ParameterManager::CB_action_success( const actionlib::SimpleClientGoalState& state, const parameter_manager::parameterResultConstPtr& result )
{
	ROS_INFO("ParameterManager::CB_action_success");
	sendResult( true );
	smc_->cancelAllClients();
}

void ParameterManager::CB_action_fail( const actionlib::SimpleClientGoalState& state, const parameter_manager::parameterResultConstPtr& result )
{
	ROS_INFO("ParameterManager::CB_action_fail");
	ROS_INFO("number of busy clients: %d", smc_->getNumberOfBusyClients());
	// Last client has sent cancel
	if ( not smc_->hasBusyClients() )
		sendResult( false );
}

// Server callbacks
void ParameterManager::sendResult( bool succes )
{
	ROS_INFO("ParameterManager::sendResult::begin");
	
	parameter_manager::parameterResult result;
	smc_->sendServerResult( succes, result );

	ROS_INFO("ParameterManager::sendResult::end");
}

void ParameterManager::CB_serverWork( const parameter_manager::parameterGoalConstPtr& goal, SMC* smc )
{
	ROS_INFO("ParameterManager::CB_serverWork::begin");
	if ( smc_->hasBusyClients() )
		smc_->cancelAllClients();

	smc_->sendGoals<ParameterAction>(*goal, clients_);

	ROS_INFO("ParameterManager::CB_serverWork::end");
}

void ParameterManager::CB_serverCancel( SMC* smc )
{
	ROS_INFO("ParameterManager::Cancel received");
}