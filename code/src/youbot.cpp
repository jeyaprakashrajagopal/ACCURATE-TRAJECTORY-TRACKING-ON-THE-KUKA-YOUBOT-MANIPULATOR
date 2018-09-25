/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: youbot.cpp
 * Path			: /src
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Sven Schneider, Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov
 * Explanation 	: Source file to access the youbot manipulator
 *************************************************************************************/
#include "youbot.hpp"

/**
 * Constructor definition : 
 * Args: number of joints
 **/
Youbot::Youbot() : numberOfJoints_(constants::MANIPULATOR_JNTS) {
    getAngles_.resize(numberOfJoints_);
    getVelocities_.resize(numberOfJoints_);
    getTorques_.resize(numberOfJoints_);
	pParamCurrentControl_value.resize(numberOfJoints_, 0.0);
	iParamCurrentControl_value.resize(numberOfJoints_, 0.0);
	dParamCurrentControl_value.resize(numberOfJoints_, 0.0);    
}

/**
 * To get the joint position / angle from the encoders of all the joints of the manipulator: 
 * Args: NA
 * Output: jointPositions
 **/
void Youbot::setJointPosition(std::vector<double> jointPositions) const {
	std::vector<youbot::JointAngleSetpoint> commandedJointPositions;
    commandedJointPositions.resize(numberOfJoints_);
   
    for (int index = 0; index < numberOfJoints_; index++) {
        commandedJointPositions[index].angle = jointPositions[index] * radians;
    }
    
	youBotArm->setJointData(commandedJointPositions);
}

/**
 * To get the joint position / angle from the encoders of all the joints of the manipulator: 
 * Args: NA
 * Output: jointPositions
 **/
std::vector<double> Youbot::getJointPosition() const {
    std::vector<double> measuredPositions(numberOfJoints_);
	youBotArm->getJointData(getAngles_);
    // Converting the KUKA joint angles to angles
    for (unsigned int index = 0; index < numberOfJoints_; index++) {
        measuredPositions[index] = quantity_cast<double>(getAngles_[index].angle);
    }
    
    return measuredPositions;
}

/**
 * To get the joint velocity from the encoders of all the joints of the manipulator: 
 * Args: NA
 * Output: jointVelocities
 **/
std::vector<double> Youbot::getJointVelocity() const {
    std::vector<double> measuredVelocities(numberOfJoints_);
	youBotArm->getJointData(getVelocities_);    // Converting the KUKA joint angles to angles
    for (unsigned int index = 0; index < numberOfJoints_; index++) {
        measuredVelocities[index] = quantity_cast<double>(getVelocities_[index].angularVelocity);
    }
    
    return measuredVelocities;
}

/**
 * To get the joint torque from the encoders of all the joints of the manipulator: 
 * Args: NA
 * Output: jointTorque
 **/
std::vector<double> Youbot::getJointTorque() const {
    std::vector<double> measuredTorques(numberOfJoints_);
	youBotArm->getJointData(getTorques_);
    // Converting the KUKA joint angles to angles
    for (int index = 0; index < numberOfJoints_; index++) {
        measuredTorques[index] = quantity_cast<double>(getTorques_[index].torque);
    }
    
    return measuredTorques;
}

/**
 * To set the joint torque to all the joints of the manipulator: 
 * Args: Vector of jointTorque
 * Output: NA
 **/
bool Youbot::setJointTorque(const std::vector<double> &jointTorque) {
    assert(jointTorque.size() == numberOfJoints_);    
	// Declaration of vector of torque setpoints to achieve the given configuration
	std::vector<youbot::JointTorqueSetpoint> manipulatorTorque;
    manipulatorTorque.resize(numberOfJoints_);
   
    for (int index = 0; index < numberOfJoints_; index++) {
        manipulatorTorque[index].torque = jointTorque[index] * newton_meter;
    }
    // Setting the torque on all the joints of the manipulator
    youBotArm->setJointData(manipulatorTorque);
    
    return true;
}

/**
 * To set the joint velocity to all the joints of the manipulator: 
 * Args: Vector of jointVelocity
 * Output: NA
 **/
void Youbot::setJointVelocity(const std::vector<double> &jointVelocity)
{
	assert(jointVelocity.size() == numberOfJoints_);
	// Declaration of vector of velocity setpoints to achieve the given configuration
	std::vector<youbot::JointVelocitySetpoint> manipulatorVelocity;
    manipulatorVelocity.resize(numberOfJoints_);
   
    for (int index = 0; index < numberOfJoints_; index++) {
        manipulatorVelocity[index].angularVelocity = jointVelocity[index] * radians_per_second;
    }
    // Setting the velocity on all the joints of the manipulator
    youBotArm->setJointData(manipulatorVelocity);
}


/**
 * Get the acceleration from the robot's joints. 
 */
std::vector<double> Youbot::getJointAcceleration() const
{
	
}

std::vector<double> Youbot::clampJointTorque(const std::vector<double> &torque) const
{
	return torque;
}

/**
 * To initialize and calibrate the manipulator: 
 * Args: NA
 * Output: NA
 **/
void Youbot::initialize()
{
    // Setting the path for the manipulator configuration file
 	youBotArm = new youbot::YouBotManipulator("youbot-manipulator", "/home/jp/youbot_driver/config/");
    // Commutate with the joints
    youBotArm->doJointCommutation();
    // Calibrating the manipulator
    youBotArm->calibrateManipulator();
	std::cout << "Youbot initialization is successful! " << std::endl;

	for(int index = 0; index < constants::MANIPULATOR_JNTS; index++)
	{
		// Proportional gain of the joint level controller
		youBotArm->getArmJoint(index + 1).getConfigurationParameter(pParamCurrentControl);
		pParamCurrentControl.getParameter(pParamCurrentControl_value[index]);
		// Integral gain of the joint level controller
		youBotArm->getArmJoint(index + 1).getConfigurationParameter(iParamCurrentControl);
		iParamCurrentControl.getParameter(iParamCurrentControl_value[index]);
		// Derivative gain of the joint level controller
		youBotArm->getArmJoint(index + 1).getConfigurationParameter(dParamCurrentControl);
		dParamCurrentControl.getParameter(dParamCurrentControl_value[index]);
	}
	
	std::cout << "Proportional gain : ";
	for(auto pParam : pParamCurrentControl_value)
		std::cout << pParam << " ";
	std::cout << "\nIntegral gain : ";
	for(auto iParam : iParamCurrentControl_value)
		std::cout << iParam << " ";
	std::cout << "\nDerivative gain : ";
	for(auto dParam : dParamCurrentControl_value)
		std::cout << dParam << " ";
	std::cout << std::endl;
}

/**
 * For the sake of the interface in this project: 
 * Args: NA
 * Output: NA
 **/
void Youbot::update(){
	
}
