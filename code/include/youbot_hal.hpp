/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: youbot_hal.hpp
 * Path			: /include
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Sven Schneider, Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov
 * Explanation 	: Header file of the interface
 *************************************************************************************/
#ifndef __YOUBOT_HAL__
#define __YOUBOT_HAL__

#include <iostream>
#include <vector>

class YoubotHal
{
	public:
		/**
		 * Virtual destructor definition
		 */
		virtual ~YoubotHal() {}
		/**
		 * Initializing the youbot manipulator joints including the calibration
		 */
		virtual void initialize() = 0;
		/**
		 * Updating the state
		 */
		virtual void update() = 0;
		/**
         * Get the position from the robot's joints. 
         */
		virtual std::vector<double> getJointPosition() const = 0;
		/**
         * Get the velocity from the robot's joints. 
         */
		virtual std::vector<double> getJointVelocity() const = 0;
		/**
         * Get the acceleration from the robot's joints. 
         */
		virtual std::vector<double> getJointAcceleration() const = 0;
		/**
         * Get the torque from the robot's joints. 
         */
		virtual std::vector<double> getJointTorque() const = 0;
		/**
         * Send new torque commands to the robot's joints. The size of the input
         * vector must match the robot's number of joints.
         */
		virtual bool setJointTorque(const std::vector<double> &torques) = 0;
		/**
         * Send new velocity commands to the robot's joints. The size of the input
         * vector must match the robot's number of joints.
         */
		virtual void setJointVelocity(const std::vector<double> &velocity) = 0;
		/**
         * To set the joint positions to the robot joints. 
         */
		virtual void setJointPosition(std::vector<double> jointPositions) const = 0;
		/**
         * There is no clamping happening here. For the sake of the interface 
         */
		virtual std::vector<double> clampJointTorque(const std::vector<double> &torque) const = 0;
		
};

#endif
