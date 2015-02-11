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
#ifndef ARM_DRIVING_POSITION_HPP
#define ARM_DRIVING_POSITION_HPP

#include <string>

#include "arm_controller_helper.hpp"
#include "operation_base_class/operation_base_class.hpp"

class ArmDrivingPosition : public OperationBaseClass
{
  public:
    ArmDrivingPosition(std::string name, ros::NodeHandle n);
    ~ArmDrivingPosition();

  private:
    void CB_goalReceived(const operations::basic_operationGoalConstPtr& goal, SMC* smc);
    void CB_armActionSuccess(const actionlib::SimpleClientGoalState& state,
                             const arm_controller::manipulateResultConstPtr& result);
    void CB_armActionFail(const actionlib::SimpleClientGoalState& state,
                          const arm_controller::manipulateResultConstPtr& result);

    bool armAction(const ArmController::Arms& arm, const arm_controller::manipulateGoal& goal);
    void moveArmToDrivingPosition();
    bool drivingPosition(const ArmController::Arms& arm);
    bool closeGripper(const ArmController::Arms& arm);
};

#endif  // ARM_DRIVING_POSITION_HPP
