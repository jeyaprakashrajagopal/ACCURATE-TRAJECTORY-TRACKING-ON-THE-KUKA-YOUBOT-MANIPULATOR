/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: frictionobserver.hpp
 * Path			: /include
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Header for the friction modeling
 *************************************************************************************/

#ifndef __FRICTION_OBSERVER__
#define __FRICTION_OBSERVER__

#include <iostream>
#include <cmath>
#include <vector>
#include "constants.hpp"

class FrictionObserver
{
	public:
		/*
		 * Constructor declaration
		 */
		FrictionObserver();

		/*
		 * Friction model for the youBot manipulator
		 */
		void frictionModel(const std::vector<double> &qd_d, const std::vector<double> &qd_m, std::vector<double> &tauFriction);

	private:
		// Static friction on the manipulator joints
		//const double staticFriction[constants::MANIPULATOR_JNTS] = {1.260, 0.956, 0.486, 0.300, 0.177};
		const double staticFriction[constants::MANIPULATOR_JNTS] = {1.260, 0.956, 0.486, 0.300, 0.500};
		// Kinetic friction on the manipulator joints
		//const double kineticFriction[constants::MANIPULATOR_JNTS] = {1.008, 0.765, 0.389, 0.240, 0.142};
		const double kineticFriction[constants::MANIPULATOR_JNTS] = {1.008, 0.765, 0.389, 0.240, 0.175};
		// To get the direction of the desired velocity
		int directionOfDesiredVelocity(double qd_des_sign);
};

#endif
