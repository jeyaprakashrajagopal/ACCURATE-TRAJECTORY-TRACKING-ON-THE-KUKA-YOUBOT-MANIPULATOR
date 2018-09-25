/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: movesynchronous.cpp
 * Path			: /src
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Source file to move the robot joints to a user-defined configuration 
 * 				  in position mode
 *************************************************************************************/

#include "movesynchronous.hpp"

/**
 * Constructor definition
 * Args : YoubotHal
 **/
Move_Synchronous::Move_Synchronous(YoubotHal *youbot, const int noOfJoints) : noOfJoints_(noOfJoints)
{
	 youbot_ = youbot;
}

/**
 * Function name 	: wait_for_stop
 * Args				: Handle of the robot
 * Returns 			: -
 * Explanation		: Waiting for the joints to stop moving.
 */
void Move_Synchronous::wait_for_stop()
{
	/// To get the velocity from the youbot_driver
	std::vector<double> getVel(noOfJoints_, 0.0);
	/// To set the velocity to the joints of the robot
	std::vector<double> setVel(noOfJoints_, 0.0);
	/// Setting the 0 rad/s to all the joints that is in operation
	youbot_->setJointVelocity(setVel);

	/// To check whether the robot is moving or not
	bool isMoving = true;
	// Waits till the safety controller sets the joint velocities to zero
	std::vector<bool> hasStoppedMoving(noOfJoints_, 0.0);
	/** 
	 * The following loop checks whether the velocity is less than the threshold or not. If the measured velocity is over 
	 * the threshold the joint is still at motion so the following loop runs till the joint is not moving 
	*/
	while(isMoving)
	{
		// To get the joint velocities from individual joints of the robot
		std::vector<double> joint_velocities = youbot_->getJointVelocity();
		// The first element of the vector for comparison
		bool firstElement = *hasStoppedMoving.begin();
		// To check if all the joints stopped moving
		bool flag = true;	
		
		// To make sure that the joints are not moving based on the observed velocities of the individual joints
		for(unsigned index = 0; index < noOfJoints_; index++)
		{
			// To make sure that the joints are set zero with an additional command since this operation is 
			// critical in the safety controller's point of view  
			youbot_->setJointVelocity(setVel);
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
	
	std::cout << "Joints stopped moving" << std::endl;
}

/**
 * Function name 	: go_to
 * Args				: Handle of the robot, jointConfiguration(candle in this work)
 * Returns 			: -
 * Explanation		: This function moves the joints to the given configuration and waits till the goal is reached
 */
void Move_Synchronous::go_to(const std::vector<double> &jointConfiguration)
{
	assert(jointConfiguration.size() == noOfJoints_);
	
	noOfJoints_ = jointConfiguration.size();
	bool hasReachedGoal = false;
	std::vector<double> getAngles(noOfJoints_, 0.0);
	/// Setting the joint angle on all the joints
	youbot_->setJointPosition(jointConfiguration);
	/** 
	 * The following loop checks whether the given joint configuration is achieved or not. If the position error is less than the 
	 * threshold value, it is assummed that the expected configuration is achieved. This check has been done for all the joints of the robot
	*/ 
	while (!hasReachedGoal) {
		/// Extracting the joint angles from the joints
		getAngles = youbot_->getJointPosition();
		for (unsigned int index = 0; index < noOfJoints_; index++) {
			bool hasPositiveError = ((jointConfiguration[index] - getAngles[index]) > constants::JOINT_POSITION_ERROR_THRESHOLD);
			bool hasNegativeError = ((jointConfiguration[index] - getAngles[index]) < -constants::JOINT_POSITION_ERROR_THRESHOLD);
			/// This condition is to check whether all the joints have reached its target or not in both positive and negative directions
			if (hasPositiveError || hasNegativeError) {
				break;
			}
			/// If all the joints have reached the expected then the while loop breaks 
			hasReachedGoal = true;
		}
	}
	wait_for_stop();
}
