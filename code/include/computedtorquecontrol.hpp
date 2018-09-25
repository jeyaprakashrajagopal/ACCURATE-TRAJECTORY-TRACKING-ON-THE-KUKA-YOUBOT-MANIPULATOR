/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: computedtorquecontrol.hpp
 * Path			: /include
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Header for the computed torque-control
 *************************************************************************************/

#ifndef __COMPUTED_TORQUE_CONTROL__
#define __COMPUTED_TORQUE_CONTROL__

#include <iostream>
#include <cmath>
#include <unistd.h>
#include "constants.hpp"
#include "dynamicmodel.hpp"
#include "pidcontroller.hpp"
#include "frictionobserver.hpp"
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames_io.hpp>

class ComputedTorqueControl
{
	public:
		/*
		 * Constructor declaration
		 */
		ComputedTorqueControl(const KDL::Chain &chain);
		/* 
		 * The feedback linearization controller for the youBot manipulator
		 */
		void computedTorqueControl(const std::vector<double> &qd_des, const std::vector<double> &q_m, const std::vector<double> &qd_m,
								   const std::vector<double> &error, std::vector<double> &tauP, std::vector<double> &tauV);

	private:
		/*
		 * Offset computations based on the coordinate frames
		 */
		void offsetCalculation(KDL::JntArray& jntpos);
			
		// Struct
		struct PID_Gains {
			double p, i, d;
		};
		std::vector<PID_Gains> positionPIDGains;
		std::vector<PID_Gains> velocityPIDGains;
		
		// Instance to the dynamic model
		DynamicModel *modelTorques;
		// To get the number of joints of the robot
		int numberOfJoints_;
		// Local variable for the commanded setpoints of the robot joints
		std::vector<double> commandedJointSetPoints_;
		// Position and velocity controller instances
		std::vector<PID_Controller> positionController;
		std::vector<PID_Controller> velocityController;
		// KDL model torques computation (ID solver)
		KDL::JntArray qdd, q_m, qd_m, tauModel;
		// To convert KDL::JntArray into a Vector
		std::vector<double> kdlTorquesToVector;		 
		// Measured joint positions and velocities
		std::vector<double> measuredJointPositions;
		// Kinematic chain for youBot
		KDL::Chain kinematicChain_;
		// Creating an object for the friction observer
		FrictionObserver frictionObserver;
		// To store the friction torques
		std::vector<double> tauFriction;
};

#endif
