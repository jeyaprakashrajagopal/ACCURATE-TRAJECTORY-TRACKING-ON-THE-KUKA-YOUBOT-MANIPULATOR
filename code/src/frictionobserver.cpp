/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: frictionobserver.cpp
 * Path			: /src
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Main file for the friction observer
 *************************************************************************************/

#include "frictionobserver.hpp"

/**
 * Construction definition with the initializations : 
 * Args: -
 **/
FrictionObserver::FrictionObserver() {
	
}


/**
 * To get the direction of the desired velocity of the manipulator joints
 * args: Desired velocity of the particular joint
 * Output: direction of the velocity
 **/
int FrictionObserver::directionOfDesiredVelocity(double qd_des_sign)
{
	// If negative value return -1
	if(qd_des_sign < 0.0) return -1;
	// If positive value return 1
	if(qd_des_sign > 0.0) return 1;
	
	return 0;
}

/**
 * Main function for computing the kinetic friction and return the static friction 
 * results which were identified through experimentation: 
 * args: qd_m (measured velocity),
 * output: tauFriction (in N.m)
 * note: The Joint numbers are starting from 0 for the purpose of vector variables indexing
 **/
void FrictionObserver::frictionModel(const std::vector<double> &qd_d, const std::vector<double> &qd_m, std::vector<double> &tauFriction)
{
	for(int jointNo = 0; jointNo < constants::MANIPULATOR_JNTS; jointNo++)
	{
		/* The joint is trying to overcome the static friction for start moving, the measured velocity is
		 * less than the threshold value which means that the joint must overcome the static friction */
		if(std::fabs(qd_m[jointNo]) <= 0.02)
		{
			tauFriction[jointNo] = staticFriction[jointNo] * directionOfDesiredVelocity(qd_d[jointNo]);
		}
		/* Kinetic friction: only Coulomb friction is used which is also based on the fact 
		 * that the Coulomb torque is less than the static/break-away friction */ 
		else if(std::fabs(qd_m[jointNo]) > 0.02)
		{
			tauFriction[jointNo] = kineticFriction[jointNo] * directionOfDesiredVelocity(qd_d[jointNo]);
		}
	}
}

