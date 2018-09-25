/*************************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: youbot.hpp
 * Path			: /include
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Sven Schneider, Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger
 * Explanation 	: Header file in order to access the youbot manipulator
 ************************************************************************************************/
#ifndef __YOUBOT_BASE__
#define __YOUBOT_BASE__

#include <vector>
#include "youbot_hal.hpp"
#include "constants.hpp"
#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/EthercatMaster.hpp"

class Youbot_Base : public YoubotHal
{
	public:
		// Constructor initialization for the youbot manipulator
		Youbot_Base();
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
		
		std::vector<double> clampJointTorque(const std::vector<double> &torques) const;		
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
	    youbot::YouBotBase *youBotBase;
		// Joint positions setting or extracting
		mutable std::vector<youbot::JointSensedAngle> getAngles_;
		// Joint velocity setting and extracting
		mutable std::vector<youbot::JointSensedVelocity> getVelocities_;
		// Joint torque setting and extracting
		mutable std::vector<youbot::JointSensedTorque> getTorques_;
};

#endif
