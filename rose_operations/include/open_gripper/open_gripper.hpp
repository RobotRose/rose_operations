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
#ifndef OPEN_GRIPPER_HPP
#define OPEN_GRIPPER_HPP

#include "arm_controller_helper.hpp"
#include "operation_base_class/operation_base_class.hpp"

class OpenGripper : public OperationBaseClass
{
  public:
	OpenGripper( std::string name, ros::NodeHandle n );
	~OpenGripper();

  private:
    void CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );
    void CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );

    void openAction();
    void receiveGoal( const rose_operations::basic_operationGoalConstPtr& goal, SMC* smc );
};

#endif //OPEN_GRIPPER_HPP