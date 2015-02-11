/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/02/14
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef SAY_HPP
#define SAY_HPP

#include <pwd.h>
#include <sound_play/SoundRequest.h>
#include <string>

#include "operation_base_class/operation_base_class.hpp"

using namespace sound_play;

class Say : public OperationBaseClass
{
  public:
	Say( std::string name, ros::NodeHandle n );
	~Say();

  private:
  	void CB_serverCancel( SMC* smc );

  	void receiveGoal( const operations::basic_operationGoalConstPtr& goal, SMC* smc );

  	bool cancel_;

  	ros::Publisher sound_play_pub_;
};

#endif //SAY_HPP