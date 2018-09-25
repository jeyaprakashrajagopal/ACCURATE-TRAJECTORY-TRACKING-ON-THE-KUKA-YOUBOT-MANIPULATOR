/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: constants.hpp
 * Path			: /include
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Defining my own constants file which is common in this project
 *************************************************************************************/

#ifndef __CONSTANTS__
#define __CONSTANTS__

namespace constants
{
	// Number of joints of the manipulator
	const int MANIPULATOR_JNTS(5);
	// Number of joints of the manipulator
	const int BASE_JNTS(4);
	// Joint limits minimum
	const double jointPositionMinimumLimit[MANIPULATOR_JNTS] = {0.5000, 0.5000, -4.6836, 0.4000, 0.5000};
	// Joint limits maximum
	const double jointPositionMaximumLimit[MANIPULATOR_JNTS] = {5.3922, 2.2053, 0.0000, 3.0779, 5.3469};
	// Velocity threshold of 0.8 rad/s
	const double velocityThreshold(1.9);
	// Mass of all the 8 segments of the manipulator
	const double link_mass[8] = {0.961, 1.390, 1.318, 0.821, 0.769, 0.94, 0.199, 0.010};
	//const double link_mass[8] = {0.001, 0.001, 0.001, 0.001, 0.001, 1.0, 0.001, 0.001};
	// maxJointTorque = nominalTorque(Nm) * reductionRatio(GearRatio)
	const double jointMaxTorque[MANIPULATOR_JNTS] = {12.9012, 12.9012, 8.2700, 4.1748, 1.7550};
	// For the offset calculations
	const double youbotOffset[MANIPULATOR_JNTS] = {2.949606436, 1.134464014, -2.548180708, 1.788962483, 2.879793266};
	// Limit for the velocity PID controller
	const double JOINT_VELOCITY_LIMIT_FOR_VELOCITYCONTROLLER = 1.68;
	// Joint positions error threshold
	const double JOINT_POSITION_ERROR_THRESHOLD = 0.01;
	// Joint velocity error threshold
	const double JOINT_VELOCITY_ERROR_THRESHOLD = 0.01;
	// The mode of control for simbody
	const int CONTROLLER_SIMBODY = 0;
	// The mode of control for the MANIPULATOR
	const int CONTROLLER_MANIPULATOR = 1;
	// The mode of control for the BASE
	const int CONTROLLER_BASE = 2;
}

#endif
