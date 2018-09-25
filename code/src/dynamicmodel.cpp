/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: kdlmodeltorques.cpp
 * Path			: /src
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Sven Schneider, Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov
 * Explanation 	: Source file to get the model torques of the youbot manipulator
 *************************************************************************************/

#include "dynamicmodel.hpp"

/** 
 * Constructor definition
 **/
DynamicModel::DynamicModel(const KDL::Chain &chain) : chainRobot_(chain)
{
	// Gravity vector
	KDL::Vector grav(0.0, 0.0, -9.81);

 	// Declaring and initializing the wrench of the rigid bodies
	f = KDL::Wrenches(chainRobot_.getNrOfSegments(), KDL::Wrench::Zero());
	
	idsolver = new KDL::ChainIdSolver_RNE(chainRobot_, grav);
}

/** 
 * Deallocation of storage space
 **/
DynamicModel::~DynamicModel()
{
	delete idsolver;
}

/**
 * This function aligns the orientation of the rotational inertia of the inertial frame wrt the joint frame since
 * the convention in KDL differs from the youbot store 3D model convention : 
 * Args: Rotation matrix of the joint frame, rotational inertia of the body
 * Output(s) - Orientation of the inertial frame is aligned with the joint frame
 **/
KDL::RotationalInertia operator*(const KDL::Rotation& M, const KDL::RotationalInertia& I)
{
	Eigen::Matrix3d R = Eigen::Map<const Eigen::Matrix3d>(M.data);
	KDL::RotationalInertia Ib;
	// General transform for mapping the orientation of the rotational inertia with respect to joint frame's rotation
	Eigen::Map<Eigen::Matrix3d>(Ib.data) = R.transpose() * (Eigen::Map<const Eigen::Matrix3d>(I.data) * R);
	
	return Ib;
}

/**
 * Extrinsic to instrinsic conversion : 
 * Args: Rotation(RPY), Translation
 * Output(s) - Frame description represented in the intrinsic convention
 **/
KDL::Frame DynamicModel::createTransform(
        double gamma, double beta, double alpha,
        double origin_x, double origin_y, double origin_z)
{
    // The youBot specification uses extrinsic Tait-Bryan angles with the ZYX
    // sequence
    double  ca = cos(alpha), sa = sin(alpha), 
			cb = cos(beta),  sb = sin(beta), 
			cc = cos(gamma), sc = sin(gamma);
	// Frame description in intrinsic convention
    return KDL::Frame(
            KDL::Rotation(
                            cb * cc    ,           -cb * sc     ,     sb  ,
                 sa * sb * cc + ca * sc, -sa * sb * sc + ca * cc, -sa * cb,
                -ca * sb * cc + sa * sc,  ca * sb * sc + sa * cc,  ca * cb),
            KDL::Vector(origin_x, origin_y, origin_z));
}

/**
 * To compute forward position kinematics based on KDL : 
 * Args: Joint positions(q)
 * Output(s) - T (End-effector frame)
 **/
KDL::Frame DynamicModel::forwardPositionKinematics(KDL::JntArray q)
{
	// To check the status of the forward kinematic solver
	bool kinematics_status;
	// End-effector frame for youBot's model
	KDL::Frame T;

	// Forward position kinematics solver
	KDL::ChainFkSolverPos_recursive fksolver(chainRobot_);
	// Joint to cartesian space that computes the forward position kinematics
	kinematics_status = fksolver.JntToCart(q, T);
	if(kinematics_status < 0) {
		std::cout << "Error: could not calculate forward kinematics for robocup model" << std::endl;
	}
	
	// Returns the end-effector frame
	return T;
}

/**
 * To compute the model torques based on KDL : 
 * Args: Joint positions(q), velocities(qd), acceleration(qdd) and the joint torques
 * Output(s) - Torque computed using ID solver based on KDL
 **/
void DynamicModel::kdlInverseDynamicsSolver(KDL::JntArray q, const KDL::JntArray &qd, const KDL::JntArray &qdd, KDL::JntArray &tauModel)
{
	// Inverse dynamics solver is invoked
	idsolver->CartToJnt(q, qd, qdd, f, tauModel);
	
	// Reverse the direction of torque and velocity since the joint 5 has -1 as joint internal axis
	tauModel(4) = -1 * tauModel(4);
}
