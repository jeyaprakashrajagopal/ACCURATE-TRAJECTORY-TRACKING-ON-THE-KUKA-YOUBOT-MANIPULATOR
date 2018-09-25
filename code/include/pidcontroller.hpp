/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: pidcontroller.hpp
 * Path			: /include
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Header for the PID-control for a joint
 *************************************************************************************/

#ifndef __PID_CONTROLLER__
#define __PID_CONTROLLER__

#include <iostream>

class PID_Controller
{
	public:
		/*
		 * Constructor declaration
		 */
		PID_Controller(double pGain, double iGain, double dGain);

		/*
		 * Computing the PI control terms
		 */
		void control(const double &error, double &controlVariable);

	private:
		// Declaration of the proportional and derivative gain vectors
		double proportionalGain;
		double derivativeGain;
		double integralGain;
		double previousError;
		double I;
};

#endif
