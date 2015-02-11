/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*  Author: Mathijs de Langen
*  Date  : 2014/04/10
*     - File created.
*
* Description:
*  description
* 
***********************************************************************************/
#ifndef CLOSE_GRIPPER_HPP
#define CLOSE_GRIPPER_HPP

#include "arm_controller_helper.hpp"
#include "operation_base_class/operation_base_class.hpp"

class CloseGripper : public OperationBaseClass
{
  public:
	CloseGripper( std::string name, ros::NodeHandle n );
	~CloseGripper();

  private:
    void CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );
    void CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );

    void closeAction();
    void receiveGoal( const operations::basic_operationGoalConstPtr& goal, SMC* smc );
};

#endif //CLOSE_GRIPPER_HPP