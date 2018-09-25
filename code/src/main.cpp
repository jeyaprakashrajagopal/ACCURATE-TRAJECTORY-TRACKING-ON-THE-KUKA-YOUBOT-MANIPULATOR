/* Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Version 1.0
* */
#include <iostream>
#include <boost/timer.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include "pidcontroller.hpp"
#include "safetycontroller.hpp"
#include "dynamicmodel.hpp"
#include "youbot_hal.hpp"
#include "youbot.hpp"
#include "computedtorquecontrol.hpp"
#include "movesynchronous.hpp"
#include "kinematicchain.hpp"

/**
 * This application demonstrates model based controller
 **/
int main(int argc, char *argv[])
{
	// Declaring an object for the interface
	YoubotHal *youbotHal = NULL;
	// Check whether the individual joints have reached the setpoints or not
	std::vector<bool> hasReached;
	// To store the Controller output for all the joints
	std::vector<double> tauOutputFromPositionController, positionError, tauOutputFromVelocityController;	
	// To get the torques from the manipulator joints
	std::vector<double> torques(constants::MANIPULATOR_JNTS, 0.0);

	// Kinematic chain for the robot
	KDL::Chain kinematicChain;
	// Object for the youbot manipulator
	KinematicChain chainYoubotManipulator;
	// Getting the kinematic chain with the call by reference
	chainYoubotManipulator.kinematicChainForYoubotManipulator(kinematicChain);

	// The desired candle configuration's vector declaration
	std::vector< double > candleConfig;
	/// Candle configuration of the youbot manipulator (in radians)
	candleConfig.push_back( 2.96244);
	candleConfig.push_back( 1.04883);
	candleConfig.push_back(-2.43523);
	candleConfig.push_back( 1.73184);
	candleConfig.push_back( 2.91062);

	// Position setpoints (target configuration) for all the joints of the manipulator
	std::vector<double> positionSetPoints = {2.96244, 1.04883, -2.43523, 1.73184, 2.9106};

	// To check whether the setpoints cross the minimum or maximum limits of the joints
	for(unsigned int jntNo = 0; jntNo < constants::MANIPULATOR_JNTS; jntNo++)
	{
		if((positionSetPoints[jntNo] <= constants::jointPositionMinimumLimit[jntNo]) || (positionSetPoints[jntNo] >= constants::jointPositionMaximumLimit[jntNo]))
		{
			std::cout << "Breach in the limits on Joint - " << (jntNo + 1) << std::endl;
			std::cout << "Please provide the setpoints within the limits of the manipulator joints " << std::endl;
		}
	}

	// Desired velocity (target configuration) for all the joints
	std::vector<double> qd_desired = {0.0, 0.0, 0.0, 0.0, 0.0};
	// Desired acceleration (target configuration) for all the joints
	std::vector<double> qdd_desired = {0.0, 0.0, 0.0, 0.0, 0.0};
	// Invoking the interface to control the youbot manipulator
	youbotHal = new Youbot();
	// To check the commanded torques before applying it to the joints
	SafetyController safetyControl(youbotHal, constants::MANIPULATOR_JNTS, constants::CONTROLLER_MANIPULATOR);
	
	// Check whether the torque limit has reached or not
	bool hasReachedGoal = false;
	// Flag that holds the control in applying the commanded torques to the joints
	bool isSafeForTheArm = true;

	// Initializes and calibrates the manipulator joints
	safetyControl.initialize();

	// Object for the Move_Synchronous class
	Move_Synchronous moveManipulatorJoints(youbotHal, constants::MANIPULATOR_JNTS);
	// Move the arm to a particular config (candle)
	moveManipulatorJoints.go_to(candleConfig);
	
	// Model based controller
	ComputedTorqueControl computedTrqCtrl(kinematicChain);

	// It containts the errors for all joints of the base
	positionError.resize(constants::MANIPULATOR_JNTS, 0.0);

	double q_desired_analyticalTime = 0.0, q_desired_analyticalTime_prev = 0.0;
	// To get the time elapsed when getting the joint positions	
	boost::timer timer;
	// Measured joint positions and velocities
	std::vector<double> measuredJointPositions, measuredJointVelocities, measureJointTorques;
	
	// Control output from the velocity PID controller 
	tauOutputFromPositionController.resize(constants::MANIPULATOR_JNTS, 0.0);
	// Control output from the velocity PID controller 
	tauOutputFromVelocityController.resize(constants::MANIPULATOR_JNTS, 0.0);
	// To check if all the joints have reached the setpoints
	hasReached.resize(constants::MANIPULATOR_JNTS);
	// For tuning the velocity pid controller
	/*std::vector<double> velocitySetpoints = {0.0, 0.0, 0.0, 1.53, 0.0}; 
	std::vector<double> velocityError(constants::MANIPULATOR_JNTS);*/
	double Tf = 10;

	while(!hasReachedGoal)
	{
		auto start = std::chrono::high_resolution_clock::now();
		// To get the joint velocities from the base joints
		measuredJointVelocities = safetyControl.getJointVelocity();
		// To get the joint positions from the base joints
		measuredJointPositions = safetyControl.getJointPosition();
		
		// For Joint 1 - analytical time-based trajectory generation using sine wave
		/*positionSetPoints[0] = (5.3922 - 2.96244) * sin(2*M_PI/Tf*q_desired_analyticalTime) + 2.96244;
		qd_desired[0] = (5.3922 - 2.96244) * 2 * M_PI / Tf * cos(2 * M_PI / Tf * q_desired_analyticalTime);*/
		// For Joint 4 - analytical time-based trajectory generation using sine wave
		positionSetPoints[3] = (3.0779 - 1.73184) * sin(2 * M_PI / Tf * q_desired_analyticalTime) + 1.73184;
		qd_desired[3] = (3.0779 - 1.73184) * 2 * M_PI / Tf * cos(2 * M_PI / Tf * q_desired_analyticalTime);
		// For Joint 5 - analytical time-based trajectory generation using sine wave
		/*positionSetPoints[4] = (5.3469 - 2.9106) * sin(2 * M_PI / Tf * q_desired_analyticalTime) + 2.9106;
		qd_desired[4] = (5.3469 - 2.9106) * 2 * M_PI / Tf * cos(2 * M_PI / Tf * q_desired_analyticalTime);*/

		for(int index = 0; index < constants::MANIPULATOR_JNTS; index++) {
			// Error (e) computation that is being sent to the controller
			positionError[index] = (positionSetPoints[index] - measuredJointPositions[index]);
			// Just for tuning the velocity pid controller
			//velocityError[index] = (velocitySetpoints[index] - measuredJointVelocities[index]);	

			// Check whether the individual joints of the base have reached the set point or not
			hasReached[index] = ( std::fabs(positionError[index]) < 0.01 );
		}
		// Invoking the model based controller
		computedTrqCtrl.computedTorqueControl(qd_desired, measuredJointPositions, measuredJointVelocities, positionError, 
											  tauOutputFromPositionController, tauOutputFromVelocityController);
		// For tuning the velocity pid controller
		/* computedTrqCtrl.computedTorqueControl(qd_desired, measuredJointPositions, measuredJointVelocities, velocityError, 
												tauOutputFromPositionController, tauOutputFromVelocityController);*/

		// If it is safe to set the joint torque to the manipulator or not
		if(isSafeForTheArm)
		{
			isSafeForTheArm = safetyControl.setJointTorque(tauOutputFromVelocityController);
		}
		//If not safe stop applying torques and break out of this loop
		else {
			break;
		}
		// To store the time lapsed in each cycle of the controller
		q_desired_analyticalTime = timer.elapsed();
		// To get the difference in time in microseconds
		q_desired_analyticalTime_prev = q_desired_analyticalTime - q_desired_analyticalTime_prev;
		auto diff = std::chrono::high_resolution_clock::now() - start;
		auto usec = std::chrono::duration_cast<std::chrono::microseconds>(diff);

		while(usec.count() <= 1000){
			diff =  std::chrono::high_resolution_clock::now() - start;
			usec = std::chrono::duration_cast<std::chrono::microseconds>(diff);
		}
	}
	
	std::vector<double> p = safetyControl.getJointPosition();
	std::cout << "Positions :";
	for(auto i : p)
		std::cout << i << " ";
	std::cout << std::endl;

	// Deallocation of the objects memory
	delete youbotHal;

	return 0;
}
