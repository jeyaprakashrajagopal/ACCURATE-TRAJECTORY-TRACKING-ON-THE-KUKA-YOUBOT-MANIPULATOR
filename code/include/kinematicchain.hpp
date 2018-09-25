/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: kinematicchain.hpp
 * Path			: /include
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Header for the kinematic chain 
 *************************************************************************************/

#ifndef __KINEMATIC_CHAIN__
#define __KINEMATIC_CHAIN__

#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames_io.hpp>
#include "constants.hpp"

class KinematicChain
{
	public:
		/*
		 * Constructor declaration
		 */
		KinematicChain() {};
		/* 
		 * Returns the kinematic chain of the youBot manipulator
		 */
		void kinematicChainForYoubotManipulator(KDL::Chain &chainRobot);
		/*
		 * Three-link manipulator
		 */
		 void threeLinkManipulator(KDL::Chain &chainRobot);

	private:
		/*
		 * Extrinsic to instrinsic conversion : 
		 */	
		KDL::Frame createTransform(double gamma, double beta, double alpha,
								   double origin_x, double origin_y, double origin_z);
};

#endif
