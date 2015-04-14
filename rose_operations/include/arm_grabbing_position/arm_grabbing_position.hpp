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

#include "arm_pose_base_class/arm_pose_base.hpp"

class ArmGrabbingPosition : public ArmPoseBaseClass
{
  public:
    ArmGrabbingPosition(std::string name, ros::NodeHandle n);
    ~ArmGrabbingPosition();
};

#endif  // ARM_GRABBING_POSITION_HPP
