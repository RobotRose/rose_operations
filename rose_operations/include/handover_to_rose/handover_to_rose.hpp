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
#ifndef HANDOVER_TO_ROSE_HPP
#define HANDOVER_TO_ROSE_HPP

#include "arm_controller_helper.hpp"
#include "operation_base_class/operation_base_class.hpp"

class HandoverToRose : public OperationBaseClass
{
  public:
	HandoverToRose( std::string name, ros::NodeHandle n );
	~HandoverToRose();

  private:
    void CB_goalReceived( const rose_operations::basic_operationGoalConstPtr& goal, SMC* smc );

    void CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );
    void CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );

    // bool handoverToRoseAction(  );
    // ArmController::Arms pickArm();
    // bool armAction( const ArmController::Arms arm, const arm_controller::manipulateGoal goal );
    // bool openGripper( const ArmController::Arms arm );
};

#endif //HANDOVER_TO_ROSE_HPP
