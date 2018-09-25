/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: pidcontroller.cpp
 * Path			: /src
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Main file for the PID controller for a joint
 *************************************************************************************/

#include "pidcontroller.hpp"

/**
 * Construction definition with the initializations : 
 * Args: The handle of the YoubotHal class(Object has been created in main.cpp)
 * Output(s) - None
 **/
PID_Controller::PID_Controller(double pGain, double iGain, double dGain) : proportionalGain(pGain), integralGain(iGain), derivativeGain(dGain) {
	I = 0.0;
	previousError = 0.0;
}

/**
 * Main function for computing the PI control variable : 
 * args: error(E) = SetPoint(SP) - ProcessVariable(PV)
 * 		 controlVariable
 **/
void PID_Controller::control(const double &error, double &controlVariable)
{
	// Reset the P and D vectors to 0.0. I should keep the accumulated values so, it is initialized in
	// the constructor
	double P = 0.0;
	double D = 0.0;
	// P value calculation
	P = error;
	// Accumulates all the error (storing the history of errors)
	I += error;
	double iErrorLimit = 1.5;
	if (I > iErrorLimit) I =  iErrorLimit;
	if (I < -iErrorLimit) I = -iErrorLimit;
	// computes the derivative term based on the current and previous error
	D = (error - previousError);

	// combining the P, I and D process variables together
	controlVariable = (proportionalGain * P) + 
					  (integralGain     * I) + 
					  (derivativeGain   * D);
	
	// To store the previous error
	previousError = error;
}

