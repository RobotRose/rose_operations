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
#ifndef ARM_POSE_BASE_CLASS_HPP
#define ARM_POSE_BASE_CLASS_HPP

#include <string>
#include <thread>

#include "operation_base_class/operation_base_class.hpp"

#include "rose_arm_controller_msgs/set_gripper_widthAction.h"
#include "rose_arm_controller_msgs/set_gripper_widthGoal.h"
#include "rose_arm_controller_msgs/set_gripper_widthFeedback.h"
#include "rose_arm_controller_msgs/set_gripper_widthResult.h"

#include "rose_arm_controller_msgs/set_positionAction.h"
#include "rose_arm_controller_msgs/set_positionGoal.h"
#include "rose_arm_controller_msgs/set_positionFeedback.h"
#include "rose_arm_controller_msgs/set_positionResult.h"

class ArmPoseBaseClass : public OperationBaseClass
{
  public:
    ArmPoseBaseClass(std::string name, ros::NodeHandle n);
    ~ArmPoseBaseClass();
    
  protected:
    void setCartesianGoal( const std::string& arm_name, const geometry_msgs::PoseStamped& goal );
    
  private:
    bool closeGripper( const std::string& arm_name );
    bool sendCartesianGoal( const std::string& arm_name, const geometry_msgs::PoseStamped& goal_pose );
    void moveArms();

    void sendWaitingMessage();

    void CB_goalReceived(const rose_operations::basic_operationGoalConstPtr& goal, SMC* smc);

    std::atomic<bool> send_message_;
    bool              cartesian_goal_set_;

    // To store goals for each arm
    std::map<std::string,geometry_msgs::PoseStamped> cartesian_goal_;
};

#endif  // ARM_POSE_BASE_CLASS_HPP
