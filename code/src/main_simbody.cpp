/* Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Version 1.0
* */
#include <iostream>
#include "kinematicchain.hpp"
#include "dynamicmodel.hpp"
#include "youbot_hal.hpp"
#include "simbody.hpp"
#include "safetycontroller.hpp"
#include "kinematicchain.hpp"

/**
 * This application demonstrates the simulated
 * youBot from its initial configuration to a desired, static configuration.
 **/
int main(int argc, char *argv[])
{
	// Kinematic chain for the robot
	KDL::Chain kinematicChain;
	// Object for the youbot manipulator
	KinematicChain chainYoubotManipulator;	
	// Getting the kinematic chain with the call by reference
	chainYoubotManipulator.kinematicChainForYoubotManipulator(kinematicChain);
	// Declaring an object for the interface
	YoubotHal *youbotHal = new Simbody(0.001, true);
	
	SafetyController safetyControllerLayer (youbotHal, constants::MANIPULATOR_JNTS, constants::CONTROLLER_SIMBODY);
	// candle mode
	KDL::JntArray q_in, qdot_in, qddot_in, tauM;
	q_in.resize(constants::MANIPULATOR_JNTS);
	qdot_in.resize(constants::MANIPULATOR_JNTS);
	qddot_in.resize(constants::MANIPULATOR_JNTS);
	tauM.resize(constants::MANIPULATOR_JNTS);
	
	// Candle
	q_in(0) = 2.949606436;
	q_in(1) = 1.134464014;
	q_in(2) = -2.548180708;
	q_in(3) = 1.788962483;
	q_in(4) = 2.879793266;

	std::vector<double> jointTorques(constants::MANIPULATOR_JNTS);

	// Model based torques computation
	DynamicModel dynamicModel(kinematicChain);
		
	while(true)
	{
		// Intializing the robot manipulator
		safetyControllerLayer.initialize();
		// model torques computation
		dynamicModel.kdlInverseDynamicsSolver(q_in, qdot_in, qddot_in, tauM);
		
		for(int i = 0; i < constants::MANIPULATOR_JNTS; i++)
			jointTorques[i] = tauM(i);
		// Setting the torque to the youbot manipulator
		safetyControllerLayer.setJointTorque(jointTorques);
	}
	
	delete youbotHal;
	
	return 0;
}
