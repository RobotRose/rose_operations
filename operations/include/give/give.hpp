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
#ifndef GIVE_HPP
#define GIVE_HPP

#include "arm_controller_helper.hpp"
#include "operation_base_class/operation_base_class.hpp"

class Give : public OperationBaseClass
{
  public:
	Give( std::string name, ros::NodeHandle n );
	~Give();

  private:
    void receiveGoal( const operations::basic_operationGoalConstPtr& goal, SMC* smc );

    void CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );
    void CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );

    void executeItemAction( const std::string item_id, const std::vector<std::string> parameters = std::vector<std::string>() );
    void continueItemAction( const std::string item_id, const std::vector<std::string> parameters = std::vector<std::string>() );
    void giveAction( const std::string item_id, const std::vector<std::string> parameters = std::vector<std::string>() );
};

#endif //GIVE_HPP