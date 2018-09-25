/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: computedtorquecontrol.cpp
 * Path			: /src
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Main file for the computed torque-control
 *************************************************************************************/

#include "computedtorquecontrol.hpp"

/**
 * Constructor definition with the initializations : 
 * Args: set points and number of joints
 **/
ComputedTorqueControl::ComputedTorqueControl(const KDL::Chain &chain) {
	// Getting the copy of the kinematic chain of the robot
	kinematicChain_ = chain; 
	// Number of joints from the kinematic chain
	numberOfJoints_ = kinematicChain_.getNrOfJoints();

	// Position PID controller gains
	positionPIDGains.resize(numberOfJoints_);
	//positionPIDGains[0] = {.p = 2.90, .i = 0.46, .d = 0.00}; // Joint 1
	positionPIDGains[0] = {.p = 0.10, .i = 0.00, .d = 0.00}; // Joint 1
	positionPIDGains[1] = {.p = 0.10, .i = 0.00, .d = 0.00}; // Joint 2
	positionPIDGains[2] = {.p = 0.10, .i = 0.00, .d = 0.00}; // Joint 3
	positionPIDGains[3] = {.p = 0.82, .i = 0.063, .d = 0.00}; // Joint 4
	positionPIDGains[4] = {.p = 1.90, .i = 0.033, .d = 0.00}; // Joint 5
	
	// Velocity PID controller gains
	velocityPIDGains.resize(numberOfJoints_);
	velocityPIDGains[0] = {.p = 0.10, .i = 0.00, .d = 0.00}; // Joint 1
	velocityPIDGains[1] = {.p = 0.10, .i = 0.00, .d = 0.00}; // Joint 2
	velocityPIDGains[2] = {.p = 0.10, .i = 0.00, .d = 0.00}; // Joint 3
	velocityPIDGains[3] = {.p = 2.90, .i = 0.18, .d = 0.00}; // Joint 3
	velocityPIDGains[4] = {.p = 1.50, .i = 0.12, .d = 0.00}; // Joint 5
	
	for(int index = 0; index < numberOfJoints_; index++) {
		// Objects creation for the position controller for all the joints
		positionController.push_back(PID_Controller(positionPIDGains[index].p, positionPIDGains[index].i, positionPIDGains[index].d));
		// Objects creation for the velocity controller for all the joints
		velocityController.push_back(PID_Controller(velocityPIDGains[index].p, velocityPIDGains[index].i, velocityPIDGains[index].d));
	}
	// Dynamic model object memory allocation
	modelTorques = new DynamicModel(kinematicChain_);
	// KDL::JntArray initilization
	q_m.resize(numberOfJoints_);
	qd_m.resize(numberOfJoints_);
	qdd.resize(numberOfJoints_);
	tauModel.resize(numberOfJoints_);
	// Friction torques vector initialization 
	tauFriction.resize(numberOfJoints_, 0.0);
}

/**
 * This function limits the velocity of the robot joints since the velocity of the 
 * position PID controller is high in most cases when applying torque:
 * Args: controller's velocity
 * Output(s) - Velocity after limiting
 **/
double limitVelocityForVelocityController(double velocity)
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
 * To compute the offset from the Kinematic to youBot coordinate frame : 
 * Args: Joint positions
 * Output(s) - Offset
 **/
void ComputedTorqueControl::offsetCalculation(KDL::JntArray& jntpos)
{
	KDL::JntArray jointpositions(constants::MANIPULATOR_JNTS);
	for(int index = 0; index < constants::MANIPULATOR_JNTS; index++)
	{
		jointpositions(index) = (jntpos(index) - constants::youbotOffset[index]);
	}

	jntpos = jointpositions;
}


/**
 * Feedback linearization control method with the cooperation of the dynamic model
 * and the controller :
 * Args: vector of errors(for all the joints)
 * Output(s) - Position and velocity control outputs
 **/
void ComputedTorqueControl::computedTorqueControl(const std::vector<double> &qd_des, const std::vector<double> &q_meas, const std::vector<double> &qd_meas, const std::vector<double> &error, 
												  std::vector<double> &velocityOutputFromPositionController, std::vector<double> &tauOutputFromVelocityController)
{
	// Cascaded position and velocity PID controller that takes errors as an input to control the individual joints of the manipulator
	for(int index = 0; index < numberOfJoints_; index++)
	{
		// Position PID control
		positionController[index].control(error[index], velocityOutputFromPositionController[index]);
		velocityOutputFromPositionController[index] = limitVelocityForVelocityController(velocityOutputFromPositionController[index] + qd_des[index]);
		// Velocity PID control and the resultant torque will be sent back to the application
		velocityController[index].control((velocityOutputFromPositionController[index] - qd_meas[index]), tauOutputFromVelocityController[index]);
		// for position pid tuning
		//velocityOutputFromPositionController[index] = limitVelocityForVelocityController(velocityOutputFromPositionController[index]);
		// Enable it for tuning the velocity PID controller
		//velocityController[index].control(limitVelocityForVelocityController(error[index]), tauOutputFromVelocityController[index]);
	}
		
	// The model based controller setup
	// std::vector to KDL::JntArray
	for(int i = 0; i < numberOfJoints_; i++) {
		q_m(i) = q_meas[i];
		qd_m(i) = qd_meas[i];
		qdd(i) = 0.0;//tauOutputFromVelocityController[i];
		// The basic approach feeds the control variable to the dynamic model
		// qdd(i) = tauOutputFromVelocityController[i];
	}
	// Offset calculation
	offsetCalculation(q_m);
	
	// Computes the model torques
	modelTorques->kdlInverseDynamicsSolver(q_m, qd_m, qdd, tauModel);
	// Friction observer with the basic control scheme
	/*if(std::fabs(qd_des[4]) > 0.0)
	{
		// Invoking the friction model to get the friction forces/torques
		frictionObserver.frictionModel(qd_des, qd_meas, tauFriction);
	}*/
	
	for(int i = 0; i < numberOfJoints_; i++) {
		// The torque that is going to be applied to the robot joints
		tauOutputFromVelocityController[i] += tauModel(i);

	}
}
