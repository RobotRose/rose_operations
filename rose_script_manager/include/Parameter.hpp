/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2013/12/19
* 		- File created.
*
* Description:
*	Parameters
* 
***********************************************************************************/
#ifndef PARAMETER_HPP
#define PARAMETER_HPP

#include <stdlib.h>

class Parameter
{
public:
	Parameter(int id, std::string name){ id = id_; name_ = name; };
	~Parameter();

	std::string 				get_name(){ return name_; };
	int 						get_id(){ return id_; };
	std::vector<std::string> 	get_types(){ return types_; };
	int 						get_selectable(){ return selectable_; };

	void set_types(std::vector<std::string> types){types_ = types; };
	void set_selectable(int selectable){ selectable_ = selectable; };

private:
	std::string 				name_;
	int							id_;
	std::vector<std::string> 	types_;
	int 						selectable_;	
};

#endif //PARAMETER_HPP