/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/04/10
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include "give/give.hpp"

Give::Give( std::string name, ros::NodeHandle n )
	: OperationBaseClass (name, n)
{
	startOperation();
}

Give::~Give()
{
	
}

void Give::CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
	// sendResult(true);
}

void Give::CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
	sendResult(false);
}

void Give::receiveGoal( const rose_operations::basic_operationGoalConstPtr& goal, SMC* smc )
{
	giveAction( "dummy" ); // Overriding choices for items
}

void Give::executeItemAction( const std::string item_id, const std::vector<std::string> parameters )
{
	ROS_INFO("Give::executeItemAction");
	Item item = datamanager_->get<Item>(item_id);

	if (!item.get_bounding_box().isSet())
		getParameter(item_id, PARAMETER_REQUEST::BOUNDING_BOX, boost::bind(&Give::continueItemAction, this, _1, _2));
	else
		giveAction( item_id );
}

void Give::continueItemAction( const std::string item_id, const std::vector<std::string> parameters )
{
	ROS_INFO("Give::continueItemAction");
	Item item = datamanager_->get<Item>(item_id);
	if (!item.get_bounding_box().isSet())
	{
		result_.return_code = MISSING_PARAMETER_ERROR;
		sendResult( false );
	}
	else
		giveAction ( item_id );
}

void Give::giveAction( const std::string item_id, const std::vector<std::string> parameters )
{
	// std::vector<double> arm_positions(7);
    // arm_positions[0] 	= 0;
 //    arm_positions[1] 	= M_PI/11;
 //    arm_positions[2] 	= M_PI/11;
 //    arm_positions[3] 	= 0;
 //    arm_positions[4] 	= -M_PI/9;
 //    arm_positions[5] 	= -M_PI/9;
 //    arm_positions[6] 	= 0;

	// std::vector<std::string> arms = arm_controller_helper_->getAvailableArms();
	// std::string arm = arms.at(0); //! @todo MdL: Pick correct arm

 // 	arm_controller::manipulateGoal goal = arm_controller_helper_->setJointPositionsMessage( arm, arm_positions );

 // 	smc_->sendGoal<arm_controller::manipulateAction>(goal, arm);
}