/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: safetycontroller_base.hpp
 * Path			: /include
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Header for the safety controller
 *************************************************************************************/

#ifndef __SAFETY_CONTROLLER__
#define __SAFETY_CONTROLLER__

#include <iostream>
#include <vector>
#include <unistd.h>
#include <cmath>
#include <cassert>
#include "constants.hpp"
#include "youbot_hal.hpp"

class SafetyController
{
	public:

		/**
		 * Constructor definition :
		 * args: object to invoke the safety controller
		 **/
		SafetyController(YoubotHal*, int, int);
		/**
         * Read the current position of the robot's joints.
         */
        std::vector<double> getJointPosition() const;

        /**
         * Read the current velocity of the robot's joints.
         */
        std::vector<double> getJointVelocity() const;

        /**
         * Read the current torque of the robot's joints.
         */
		std::vector<double> getJointTorque() const;

        /**
         * To initialize the youbot manipulator/simulator.
         *
         * This function will start a thread for the simulation/real-robot and immediately
         * return.
         */
		void initialize();
	
        /**
         * Send new torque commands to the robot's joints. The size of the input
         * vector must match the robot's number of joints.
         */
        bool setJointTorque(const std::vector<double> &torques);
		/** Send new velocity commands to the robot's joints. The size of the input
         * vector must match the robot's number of joints.
         */      
		void setJointVelocity(const std::vector<double> &jointVelocity);
		/** For the sake of the interface */
        void update();
		/**
         * For the interface. 
         */		
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
		// To get the number of joints
		const int numberOfJoints_;
		// Flag to check the safety
		mutable bool isSafe;
		// Object to get the shared pointer
		YoubotHal *youbot_;
		// To switch over to the velocity mode from torque mode   
		std::vector<double> setJointVelocityToZero;
		std::vector<double> initialConfiguration;
		double differenceBwInitialAndTargetConfiguration;
		std::vector<double> jointMaxTorque;

		/**To check the position of the joint is not going over the threshold value
		 * args: position array, target configuration +/-threshold value
		 **/
		bool positionVelocityTorqueControlCheck(std::vector<double> , std::vector<double> , std::vector<double> , std::vector<double> ) const;
};
#endif
