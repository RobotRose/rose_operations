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

#include "arm_pose_base_class/arm_pose_base.hpp"

class HandoverToRose : public ArmPoseBaseClass
{
  public:
	HandoverToRose( std::string name, ros::NodeHandle n );
	~HandoverToRose();
};

#endif //HANDOVER_TO_ROSE_HPP
