/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: safetycontroller.cpp
 * Path			: /src
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Main file for the safety controller
 *************************************************************************************/

#include "safetycontroller.hpp"

/**
 * Constructor definition : 
 * Args: YoubotHal object's shared pointer
 **/
SafetyController::SafetyController(YoubotHal *youbot, int noOfJoints, int modeOfControl) : numberOfJoints_(noOfJoints)
{
	assert(numberOfJoints_ == constants::BASE_JNTS || numberOfJoints_ == constants::MANIPULATOR_JNTS);
	youbot_ = youbot;
	differenceBwInitialAndTargetConfiguration = 0.0;
	// If the safety control request has made for the base of the robot
	if(modeOfControl == constants::CONTROLLER_BASE) {
		jointMaxTorque = {1.0, 1.0, 1.0, 1.0};
		initialConfiguration = {0.0, 0.0, 0.0, 0.0};
	}
	// If the safety control request has made for the manipulator
	else if(modeOfControl == constants::CONTROLLER_MANIPULATOR || modeOfControl == constants::CONTROLLER_SIMBODY) {
		jointMaxTorque = {12.9012, 12.9012, 8.2700, 4.1748, 1.7550};
		//jointMaxTorque = {12.9012, 12.9012, 8.2700, 4.1748, 1.0000};
		initialConfiguration = {2.96244, 1.04883, -2.43523, 1.73184, 2.9106};
	}
	// To set the joints of the robot to zero value
	setJointVelocityToZero.resize(numberOfJoints_, 0.0);
	// Preliminary condition of the flag
	isSafe = false;
}

/**
 * To get the joint position from the joints of the manipulator : 
 * Args: -
 * Output(s) - Returns the joint positions vector
 **/
std::vector<double> SafetyController::getJointPosition() const {
	return youbot_->getJointPosition();
}

/**
 * To set the joint positions to the robot joints.
 * Args: vector of joint positions for the manipulator 
 */
void SafetyController::setJointPosition(std::vector<double> jointPositions) const
{
	youbot_->setJointPosition(jointPositions);
}

/**
 * To get the joint velocity from the joints of the manipulator : 
 * Args: - NA
 * Output(s) - Returns the joint velocities vector
 **/
std::vector<double> SafetyController::getJointVelocity() const {
	return youbot_->getJointVelocity();
}

/**
 * To get the joint torque from the joints of the manipulator : 
 * Args: NA
 * Output(s) - Returns the joint torques vector
 **/
std::vector<double> SafetyController::getJointTorque() const {
	return youbot_->getJointTorque();
}

/**
 * Get the acceleration from the robot's joints. 
 */
std::vector<double> SafetyController::getJointAcceleration() const {
	
}

/**
 * To command the joint torque to the joints of the manipulator : 
 * Args: vector of joint torques that need to be set
 * Output(s) - NA
 **/
bool SafetyController::setJointTorque(const std::vector<double> &jointTrq)
{
	// Limit check for all the joints of the manipulator
	std::vector<double> jointTorques = jointTrq;

	// Limit check for all the joints of the manipulator
	for (int index = 0; index < numberOfJoints_; index++)
	{
		// Checking the absolute values since torque depends on the direction(either +ve/-ve)
		if(fabs(jointTorques[index]) > jointMaxTorque[index])
		{
			//std::cout << "Maximum torque limit is exceeded in the joint :" << (index + 1) << ": " << jointTorques[4] << std::endl;
			// If the torques are negative based on the direction of the joint movement
			if(jointTorques[index] < 0) {
				jointTorques[index] = -1.0 * jointMaxTorque[index];
			}
			// If the joint movement direction is positive
			else {
				jointTorques[index] = 1.0 * jointMaxTorque[index];
			}
		}
	}
	
	// If the torque is NaN(Not a Number) set torque to 0.0
	for(int index = 0; index < numberOfJoints_; index++)
	{
		if(!std::isfinite(jointTorques[index]))
		{
			jointTorques[index] = 0.0;
			//std::cout << "jointTorques[" << index << "]" << " is NaN" << std::endl;
			return false;
		}
	}

	// Is it safe or not in all the modes such as position, velocity and torque control mode?
	isSafe = positionVelocityTorqueControlCheck(initialConfiguration, getJointPosition(), 
														getJointVelocity(), getJointTorque());

	// If safe set the torque of the joints
	if(isSafe)
	{
		youbot_->setJointTorque(jointTorques);
		return true;
	}
	else {
		// Sets the velocity of the particular joint to zero in order
		// to switch from torque to velocity mode
		setJointVelocity(setJointVelocityToZero);

		// Waits till the safety controller sets the joint velocities to zero
		std::vector<bool> hasStoppedMoving(numberOfJoints_, 0.0);
		//Flag to check whether the joints of the manipulator is still at motion
		bool isMoving = true;

        while(isMoving)
        {
			// To get the joint velocities from individual joints of the manipulator
			std::vector<double> joint_velocities = getJointVelocity();
			// The first element of the vector for comparison
			bool firstElement = *hasStoppedMoving.begin();
			// To check if all the joints stopped moving
			bool flag = true;	
			
			// To make sure that the joints are not moving based on the observed velocities of the individual joints
			for(unsigned index = 0; index < numberOfJoints_; index++)
			{
				// To make sure that the joints are set zero with an additional command since this operation is 
				// critical in the safety controller's point of view  
				setJointVelocity(setJointVelocityToZero);
				// For all the joints 
				hasStoppedMoving[index] = ( (joint_velocities[index] <= 0.01) && (joint_velocities[index] >= -0.01) );
			}
			// Iterates through the vector to check the elements are equal or not
			for(std::vector<bool>::const_iterator it = hasStoppedMoving.begin() + 1; it != hasStoppedMoving.end(); ++it)
			{
				// If all the elements of the vector are not equal
				if(*it != firstElement) {
					// The flag is set to false when the elements of the vector are not the same
					flag = false;
					break;
				}
				// If all the elements of the vector are equal
				else {
					// The flag holds true if all the joints are at rest 
					flag = flag & true;
				}
			}
			// The flag will be set to true when all the joints have stopped moving
			if(flag == true) {
				isMoving =false;
				break;
			}
        }

		return false;
	}
}

/**
 * To command the joint velocity to the joints of the manipulator : 
 * Args: vector of joint velocity inputs
 * Output(s) - NA
 **/
void SafetyController::setJointVelocity(const std::vector<double> &jointVelocity) {
	// This sets the joint velocities to the manipulator joints
	youbot_->setJointVelocity(jointVelocity);
}

/**
 * To initialize the youbot manipulator and calibrate it : 
 * Args: NA
 * Output(s) - Initializes the manipulator
 **/
void SafetyController::initialize() {
	// To initialize the youbot manipulator and calibrate it
	youbot_->initialize();
}

/**
 * For the sake of interface : 
 * Args: NA
 * Output(s) - Initializes the manipulator
 **/
std::vector<double> SafetyController::clampJointTorque(const std::vector<double> &torques) const
{
	return torques;
}

/**
 * For the sake of interface created : 
 * Args: NA
 * Output(s) - NA
 **/
void SafetyController::update(){
	
}

/**
 * To check the positions of the joints of the manipulator : 
 * Args: Joint positions measured
 * Output(s) - Safe or not
 **/
bool SafetyController::positionVelocityTorqueControlCheck(std::vector<double> initialPosition, std::vector<double> measuredPosition, 
														  std::vector<double> measuredVelocity, std::vector<double> measuredTorque) const
{
	for(unsigned int index = 0; index < numberOfJoints_; index++)
	{
		// Position control safety limits check
		if((measuredPosition[index] <= constants::jointPositionMinimumLimit[index]) ||
		   (measuredPosition[index] >= constants::jointPositionMaximumLimit[index])) 
		{
			std::cout << "The joint position limit has been reached in : " << (index + 1) << " " << measuredPosition[index] << std::endl;
			return false;
		}
		
		// Velocity control safety limits check
		if(std::fabs(measuredVelocity[index]) > constants::velocityThreshold) {
			std::cout << "The joint velocity limit has been reached in the joint. " << (index + 1) << " " << measuredVelocity[index] << std::endl;
			return false;
		}

		// Torque control safety limits check TODO
		//std::cout << "Measured torque in safety controller: " << jointTorque_ << std::endl;
		if(std::fabs(measuredTorque[index]) > constants::jointMaxTorque[index]) {
			std::cout << "The joint torque limit has been reached in the joint. " << (index + 1) << " " << measuredTorque[index] << std::endl;
			return false;
		}
	}
	return true;
}
