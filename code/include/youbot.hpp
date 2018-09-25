/*************************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: youbot.hpp
 * Path			: /include
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Sven Schneider, Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov
 * Explanation 	: Header file in order to access the youbot manipulator
 ************************************************************************************************/
#ifndef __YOUBOT__
#define __YOUBOT__

#include <vector>
#include "youbot_hal.hpp"
#include "constants.hpp"
#include <youbot_driver/youbot/YouBotManipulator.hpp>
#include <youbot_driver/youbot/YouBotJointParameter.hpp>
#include <youbot_driver/youbot/YouBotJoint.hpp>

class Youbot : public YoubotHal
{
	public:
		// Constructor initialization for the youbot manipulator
		Youbot();
		// Initializes and calibrates the manipulator
		void initialize();
		// To get the vector of joint positions of the manipulator
		std::vector<double> getJointPosition() const;
		// To get the vector of joint velocities of the manipulator 
		std::vector<double> getJointVelocity() const;
		// To get the vector of joint torque of the manipulator 
		std::vector<double> getJointTorque() const;
		// To set the torque in the individual joints
		bool setJointTorque(const std::vector<double> &jointTorque);
		// Update the state
		void update();

		/* Send new velocity commands to the robot's joints. The size of the input
         * vector must match the robot's number of joints.
         */
		void setJointVelocity(const std::vector<double> &jointVelocity);
		/**
         * There is no clamping happening here. For the sake of the interface 
         */		
		std::vector<double> clampJointTorque(const std::vector<double> &torque) const;		
		/**
         * Get the acceleration from the robot's joints. 
         */
		std::vector<double> getJointAcceleration() const;
		/**
         * To set the joint positions to the robot joints. 
         */
		void setJointPosition(std::vector<double> jointPositions) const;

	private:
		// Number of joints in the manipulator
		const int numberOfJoints_;
		// Handle for the youbot manipulator
	    youbot::YouBotManipulator *youBotArm;
		// Joint positions setting or extracting
		mutable std::vector <youbot::JointSensedAngle> getAngles_;
		// Joint velocity setting and extracting
		mutable std::vector <youbot::JointSensedVelocity> getVelocities_;
		// Joint torque setting and extracting
		mutable std::vector <youbot::JointSensedTorque> getTorques_;
		// To get the proportional gain of the joint level controller
		youbot::PParameterCurrentControl pParamCurrentControl;
		std::vector<int> pParamCurrentControl_value;
		// To get the integral gain of the joint level controller
		youbot::IParameterCurrentControl iParamCurrentControl;
		std::vector<int> iParamCurrentControl_value;
		// To get the derivative gain of the joint level controller
		youbot::DParameterCurrentControl dParamCurrentControl;
		std::vector<int> dParamCurrentControl_value;
};

#endif
