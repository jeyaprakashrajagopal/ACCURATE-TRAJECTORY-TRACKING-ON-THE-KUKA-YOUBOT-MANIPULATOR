/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: dynamicmodel.hpp
 * Path			: /include
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Header for the computing the Orocos KDL model torques
 *************************************************************************************/

#ifndef __DYNAMIC_MODEL__
#define __DYNAMIC_MODEL__

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames_io.hpp>
#include "constants.hpp"
#include <Eigen/Core>
#include <map>

class DynamicModel
{
	public:
		/*
		 * Constructor declaration
		 */
		DynamicModel(const KDL::Chain &chain);
		/*
		 * Destructor declaration
		 */
		virtual ~DynamicModel();

		/*
		 * Computing model torques based on the given joint information(position, velocity and acceleration)
		 */
		void kdlInverseDynamicsSolver(KDL::JntArray, const KDL::JntArray &, const KDL::JntArray &, KDL::JntArray &);
		/*
		 * Forward kinematic solver
		 */
		KDL::Frame forwardPositionKinematics(KDL::JntArray q);
	
	private:
		/*
		 * Transform from extrinsic to instrinsic convention
		 */
		KDL::Frame createTransform(double gamma, double beta, double alpha, double origin_x, double origin_y, double origin_z);
		// torque joint array
		KDL::JntArray tau;
		// Inverse dynamics solver object instantiation
		KDL::ChainIdSolver_RNE *idsolver;
		// Kinematic chain for youBot
		KDL::Chain chainRobot_;
		// Wrench object instantiation
		KDL::Wrenches f;
};
#endif
