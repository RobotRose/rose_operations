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
#ifndef ARM_GRABBING_POSITION_HPP
#define ARM_GRABBING_POSITION_HPP

#include <string>

#include "arm_controller_helper.hpp"
#include "operation_base_class/operation_base_class.hpp"

class ArmGrabbingPosition : public OperationBaseClass
{
  public:
    ArmGrabbingPosition(std::string name, ros::NodeHandle n);
    ~ArmGrabbingPosition();

  private:
    void CB_goalReceived(const operations::basic_operationGoalConstPtr& goal, SMC* smc);
    void CB_armActionSuccess(const actionlib::SimpleClientGoalState& state,
                             const arm_controller::manipulateResultConstPtr& result);
    void CB_armActionFail(const actionlib::SimpleClientGoalState& state,
                          const arm_controller::manipulateResultConstPtr& result);

    bool armAction(const ArmController::Arms& arm, const arm_controller::manipulateGoal& goal);
    void moveArmToGrabbingPosition();
    bool grabbingPosition(const ArmController::Arms& arm);
    bool closeGripper(const ArmController::Arms& arm);
};

#endif  // ARM_GRABBING_POSITION_HPP
