/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: movesynchronous.hpp
 * Path			: /src
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Header file to move the arm to a particular configuration in position 
 * 				  mode
 *************************************************************************************/

#ifndef __MOVE_SYNCHRONOUS__
#define __MOVE_SYNCHRONOUS__

#include <iostream>
#include <vector>
#include "assert.h"
#include "constants.hpp"
#include "youbot_hal.hpp"

class Move_Synchronous
{
	public:
		/*
		 * Constructor declaration
		 */
		Move_Synchronous(YoubotHal *youbot, const int noOfJoints);
		// move the robot to the desired configuration
		void go_to(const std::vector<double> &jointConfig);
	
	private:
		// Local object for an interface
		YoubotHal *youbot_;
		// To wait till the joints stop moving
		void wait_for_stop();
		// To keep the number of joints in the manipulator
		unsigned int noOfJoints_;
};

#endif
