/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2013/12/19
* 		- File created.
*
* Description:
*	Script class
* 
***********************************************************************************/
#ifndef SCRIPT_HPP
#define SCRIPT_HPP

#include <stdlib.h>

class Script
{
public:
	Script(int id, std::string name){ id_ = id ; name_ = name; };
	~Script();

	int 		get_id(){ return id_;};
	std::string get_name(){ return name_; };
	
private:
	std::string name_;
	int			id_;
};

#endif // SCRIPT_HPP