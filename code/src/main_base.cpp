/* Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Version 1.0
* */
#include <iostream>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include "pidcontroller.hpp"
#include "safetycontroller.hpp"
#include "youbot_hal.hpp"
#include "youbot_base.hpp"
#include <boost/timer.hpp>

/**
 * This function limits the velocity of the robot joints since the velocity of the 
 * position PID controller is high in most cases when applying torque:
 * Args: controller's velocity
 * Output(s) - Velocity after limiting
 **/
double limitVelocityController(double velocity)
{
	double velocityAfterLimiting = velocity;
	
	if(velocity < -constants::JOINT_VELOCITY_LIMIT_FOR_VELOCITYCONTROLLER) {
		velocityAfterLimiting = -constants::JOINT_VELOCITY_LIMIT_FOR_VELOCITYCONTROLLER;
	}
	
	else if(velocity > constants::JOINT_VELOCITY_LIMIT_FOR_VELOCITYCONTROLLER) {
		velocityAfterLimiting = constants::JOINT_VELOCITY_LIMIT_FOR_VELOCITYCONTROLLER;
	}

	return velocityAfterLimiting;
}

/**
 * This application demonstrates a simple PD-controller to move a simulated
 * youBot from its initial configuration to a desired, static configuration.
 **/
int main(int argc, char *argv[])
{
	// Declaring an object for the interface
	YoubotHal *youbotHal = NULL;
	// Check whether the individual joints have reached the setpoints or not
	std::vector<bool> hasReached;
	// To store the Controller output for all the joints
	std::vector<double> tauOutputFromPositionController, errorPControl, tauOutputFromVelocityController;	
	// This object holds the handle for controlling the base
	youbotHal = new Youbot_Base();
	// Instance that monitors the commanded data and check those against the safety conditions
	SafetyController safetyControl(youbotHal, constants::BASE_JNTS, constants::CONTROLLER_BASE);
	// Initializes the base joints and resets the encoder
	safetyControl.initialize();

	// Torque commands of all the joints
	std::vector<double> inputTorqueToBaseJoint = {0.0, 0.0, 0.0, 0.0};
	// Set point commands of the individual joints of the base 
	std::vector<double> setPoints = {0.0, 0.0, 0.0, 0.0};
	// This flag checks the safety criterion of the safety controller to avoid the damage
	bool isSafe = true;
	
	// Position PID controller gains
	double positionPIDGains[constants::BASE_JNTS][3] = {{1, 0.0, 0.0}, {1, 0.0, 0.0}, 
														{1, 0.0, 0.0}, {1, 0.0, 0.0}};
	// Velocity PID controller gains
	double velocityPIDGains[constants::BASE_JNTS][3] = {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 
														{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
	// Position and velocity PID controller instances for the individual joints 
	std::vector<PID_Controller> positionController, velocityController;

	for(int index = 0; index < constants::BASE_JNTS; index++) {
		// Objects creation for the position controller of all the joints
		positionController.push_back(PID_Controller(positionPIDGains[index][0], positionPIDGains[index][1], positionPIDGains[index][2]));
		// Objects creation for the velocity controller of all the joints
		velocityController.push_back(PID_Controller(velocityPIDGains[index][0], velocityPIDGains[index][1], velocityPIDGains[index][2]));
	}

	// Flag to hold the responsibility in running the torque controller
	bool hasReachedGoal = false;
	// Stores the measured joint positions, time elapsed for the control operation
	std::ofstream controllerOutputJointPositions;
	// Open the file for recording the data
	controllerOutputJointPositions.open("joint4.txt");
	controllerOutputJointPositions.flush();
	// To get the time elapsed when getting the joint positions	
	boost::timer timer;

	while(!hasReachedGoal)
	{
		if(timer.elapsed() < 2)
			setPoints[3] = (1.0 - 0.0) * sin(2*M_PI/10*timer.elapsed()) + 0.0;

		// Control output from the position PID controller 
		tauOutputFromPositionController.resize(constants::BASE_JNTS, 0.0);
		// Control output from the velocity PID controller 
		tauOutputFromVelocityController.resize(constants::BASE_JNTS, 0.0);
		// It containts the errors for all joints of the base
		errorPControl.resize(constants::BASE_JNTS, 0.0);
		// To get the joint positions from the base joints
		std::vector<double> measuredJointPositions = safetyControl.getJointPosition();
		// To get the joint velocities from the base joints
		std::vector<double> measuredJointVelocities = safetyControl.getJointVelocity();
		// Check whether the individual joints have reached the setpoints or not
		hasReached.resize(constants::BASE_JNTS);

		for(int index = 0; index < constants::BASE_JNTS; index++) {
			// Error (e) computation that is being sent to the controller
			errorPControl[index] = (setPoints[index] - measuredJointPositions[index]);
			// Position PID controller is invoked
			positionController[index].control(errorPControl[index], tauOutputFromPositionController[index]);
			// Velocity PID controller is invoked
			velocityController[index].control(limitVelocityController(tauOutputFromPositionController[index]), tauOutputFromVelocityController[index]);
			// Check whether the individual joints of the base have reached the set point or not
			hasReached[index] = (measuredJointPositions[index] > (setPoints[index] - constants::JOINT_POSITION_ERROR_THRESHOLD) &&
						  measuredJointPositions[index] < (setPoints[index] + constants::JOINT_POSITION_ERROR_THRESHOLD));
		}

		// Total time elapsed to reach the setpoint and the waypoints are also recorded
		//time_elapsed = time_elapsed + timer.elapsed();
		controllerOutputJointPositions << measuredJointPositions[3] << ", " << timer.elapsed() << std::endl;
		// IF all the joints reach the setpoint stop the controller
		if(hasReached[0] && hasReached[1] && hasReached[2] && hasReached[3]) {
			//hasReachedGoal = true;
		}

		// To set the velocity contoller's output to the manipulator joints 
		inputTorqueToBaseJoint = tauOutputFromVelocityController;
		// To Check whether it is safe to set the joint torques or not
		if(isSafe) {
			isSafe = safetyControl.setJointTorque(inputTorqueToBaseJoint);
		}else {
			// Stops setting torques to the robot joints when safety limits are breached
			break;
		}
	}

	std::vector<double> p = safetyControl.getJointPosition();
	std::cout << "Positions :";
	for(auto i : p)
		std::cout << i << " ";
	std::cout << std::endl;

	controllerOutputJointPositions.close();
	
	delete youbotHal;
	return 0;
}
