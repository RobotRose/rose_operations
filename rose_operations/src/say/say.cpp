/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
* Author: Mathijs de Langen
* Date  : 2014/02/14
*     - File created.
*
* Description:
* description
* 
***********************************************************************************/
#include "say/say.hpp"

Say::Say( std::string name, ros::NodeHandle n )
	: OperationBaseClass (name, n)
{
    sound_play_pub_ = n_.advertise<SoundRequest>("/robotsound", 1, false);

    startOperation();
}

Say::~Say()
{

}

void Say::CB_serverCancel( SMC* smc )
{
	cancel_ = true;	
}

void Say::receiveGoal( const rose_operations::basic_operationGoalConstPtr& goal, SMC* smc )
{
    //homedir 
    passwd* pw = getpwuid(getuid());
    std::string path(pw->pw_dir);

    Resource audio = datamanager_->get<Resource>(goal->item_ids.at(0));
    path += audio.get_filename();
    ROS_INFO("audio path: %s", path.c_str());

	SoundRequest msg;
    msg.sound = SoundRequest::PLAY_FILE;
    msg.command = SoundRequest::PLAY_ONCE;
    msg.arg = path;
    msg.arg2 = "";
    sound_play_pub_.publish(msg);

    if (not cancel_)
    	sendResult(true);
    else
        sendResult(false);
}
