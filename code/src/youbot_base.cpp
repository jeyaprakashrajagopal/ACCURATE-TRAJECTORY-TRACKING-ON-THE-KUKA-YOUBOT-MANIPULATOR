/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: youbot_base.cpp
 * Path			: /src
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Sven Schneider, Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger
 * Explanation 	: Source file to access the youbot manipulator
 *************************************************************************************/
#include "youbot_base.hpp"

/**
 * Constructor definition : 
 * Args: NA
 **/
Youbot_Base::Youbot_Base() : numberOfJoints_(constants::BASE_JNTS) {
    getAngles_.resize(numberOfJoints_, (0.0 * radians));
    getVelocities_.resize(numberOfJoints_, (0.0 * radians_per_second));
    getTorques_.resize(numberOfJoints_, (0.0 * newton_meter));
}

/**
 * To set the joint positions to the robot joints.
 * Args: vector of joint positions for the manipulator 
 */
void Youbot_Base::setJointPosition(std::vector<double> jointPositions) const
{

}
/**
 * To get the joint position / angle from the encoders of all the joints of the manipulator: 
 * Args: NA
 * Output: jointPositions
 **/
std::vector<double> Youbot_Base::getJointPosition() const {
    std::vector<double> measuredJointPositions(numberOfJoints_, 0.0);
	youBotBase->getJointData(getAngles_);
    // Converting the KUKA joint angles to angles
	for(int index = 0; index < numberOfJoints_; index++) {
	    measuredJointPositions[index] = quantity_cast<double>(getAngles_[index].angle);
	}
    return measuredJointPositions;
}

/**
 * To get the joint velocity from the encoders of all the joints of the manipulator: 
 * Args: NA
 * Output: jointVelocities
 **/
std::vector<double> Youbot_Base::getJointVelocity() const {
    std::vector<double> measuredJointVelocities(numberOfJoints_, 0.0);
	youBotBase->getJointData(getVelocities_);
	// Converting the KUKA joint angles to angles
	for(int index = 0; index < numberOfJoints_; index++) {
    	measuredJointVelocities[index] = quantity_cast<double>(getVelocities_[index].angularVelocity);
	}
    return measuredJointVelocities;
}

/**
 * To get the joint torque from the encoders of all the joints of the manipulator: 
 * Args: NA
 * Output: jointTorque
 **/
std::vector<double> Youbot_Base::getJointTorque() const {
    std::vector<double> measuredJointTorques(numberOfJoints_, 0.0);
	youBotBase->getJointData(getTorques_);
    // Converting the KUKA joint torque to double
	for(int index = 0; index < numberOfJoints_; index++) {
		measuredJointTorques[index] = quantity_cast<double>(getTorques_[index].torque);
	}

    return measuredJointTorques;
}

/**
 * To set the joint torque to all the joints of the manipulator: 
 * Args: Vector of jointTorque
 * Output: NA
 **/
bool Youbot_Base::setJointTorque(const std::vector<double> &jointTorques) {

	// Declaration of vector of torque setpoints to achieve the given configuration
	std::vector<youbot::JointTorqueSetpoint> commandedTorques;
	commandedTorques.resize(numberOfJoints_);

	for (int index = 0; index < numberOfJoints_; index++) {
	    commandedTorques[index].torque = jointTorques[index] * newton_meter;
	}
	// Setting the torque on all the joints of the manipulator
    youBotBase->setJointData(commandedTorques);
	
	return true;
}

/**
 * To set the joint velocity to all the joints of the manipulator: 
 * Args: Vector of jointVelocity
 * Output: NA
 **/
void Youbot_Base::setJointVelocity(const std::vector<double> &jointVelocity)
{
	// Declaration of vector of velocity setpoints to achieve the given configuration
	std::vector<youbot::JointVelocitySetpoint> commandedJointVelocity;
	commandedJointVelocity.resize(numberOfJoints_);

	for(unsigned int index = 0; index < numberOfJoints_; index++) {
    	commandedJointVelocity[index].angularVelocity = jointVelocity[index] * radians_per_second;
	}

    // Setting the velocity on all the joints of the manipulator
    youBotBase->setJointData(commandedJointVelocity);
}

/**
 * Get the acceleration from the robot's joints. 
 */
std::vector<double> Youbot_Base::getJointAcceleration() const
{
	
}

std::vector<double> Youbot_Base::clampJointTorque(const std::vector<double> &torques) const
{
	return torques;
}

/**
 * To initialize and calibrate the manipulator: 
 * Args: NA
 * Output: NA
 **/
void Youbot_Base::initialize()
{
    // Setting the path for the manipulator configuration file
 	youBotBase = new youbot::YouBotBase("youbot-base", "/home/jp/youbot_driver/config/");
    // Commutate with the joints
    youBotBase->doJointCommutation();
    // Calibrating the manipulator
	/// Initializing the encoder values
	for(int index = 1; index <= numberOfJoints_; index++) {
		youBotBase->getBaseJoint(index).setEncoderToZero();
	}
	// To get the controller gains from the joint level
	/*for(int index = 0; index < constants::MANIPULATOR_JNTS; index++)
	{
		// Proportional gain of the joint level controller
		youBotBase->getArmJoint(index + 1).getConfigurationParameter(pParamCurrentControl);
		pParamCurrentControl.getParameter(pParamCurrentControl_value[index]);
		// Integral gain of the joint level controller
		youBotBase->getArmJoint(index + 1).getConfigurationParameter(iParamCurrentControl);
		iParamCurrentControl.getParameter(iParamCurrentControl_value[index]);
		// Derivative gain of the joint level controller
		youBotBase->getArmJoint(index + 1).getConfigurationParameter(dParamCurrentControl);
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
	std::cout << std::endl;	*/
}

/**
 * For the sake of the interface in this project: 
 * Args: NA
 * Output: NA
 **/
void Youbot_Base::update(){
	
}
