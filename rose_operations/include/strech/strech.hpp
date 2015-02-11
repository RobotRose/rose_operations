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
#ifndef STRECH_HPP
#define STRECH_HPP

#include "arm_controller_helper.hpp"
#include "operation_base_class/operation_base_class.hpp"

class Strech : public OperationBaseClass
{
  public:
	Strech( std::string name, ros::NodeHandle n );
	~Strech();

  private:
    void CB_goalReceived( const rose_operations::basic_operationGoalConstPtr& goal, SMC* smc );
    void CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );
    void CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );

    void strechArms();
};

#endif //STRECH_HPP