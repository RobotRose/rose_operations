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
#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <stdlib.h>

#include "Parameter.hpp"

class Parameters
{
public:
	Parameters();
	~Parameters();

	std::vector<Parameter>  get_paramters(){ return parameters_; };

private:
	std::vector<Parameter> parameters_;	
};

#endif //PARAMETER_HPP