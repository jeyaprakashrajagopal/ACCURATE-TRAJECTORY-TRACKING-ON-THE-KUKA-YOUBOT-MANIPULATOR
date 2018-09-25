/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Filename 	: kinematicchain.cpp
 * Path			: /src
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov, Sven Schneider
 * Explanation 	: Main file for getting the kinematic chain of the robots
 *************************************************************************************/

#include "kinematicchain.hpp"

/**
 * Extrinsic to instrinsic conversion : 
 * Args: Rotation(RPY), Translation
 * Output(s) - Frame description represented in the intrinsic convention
 **/
KDL::Frame KinematicChain::createTransform(
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
 * Kinematic chain of the 3-link manipulator
 * Args: chain
 * Output(s) - chainRobot (call-by-reference)
 **/
void KinematicChain::threeLinkManipulator(KDL::Chain &chainRobot)
{
	KDL::Frame frame_identity = createTransform(0.000, 0.000, 0.000, 0.000, 0.000, 0.000),
			   arm_joint_1_frame = createTransform(0.000, 0.000, M_PI/4, 0.000, 0.000, 1.000),
			   arm_joint_2_frame = createTransform(0.000, 0.000, 0.000, 0.000, 0.000, 1.000);
	
	chainRobot.addSegment(KDL::Segment("base_link", KDL::Joint(KDL::Joint::None), arm_joint_1_frame));
	chainRobot.addSegment(KDL::Segment("arm_link_1", KDL::Joint(KDL::Joint::RotZ),
				arm_joint_2_frame, KDL::RigidBodyInertia(1, arm_joint_2_frame.Inverse() * KDL::Vector(1, 1, 1), KDL::RotationalInertia(1, 1, 1, 0, 0, 0))));
	chainRobot.addSegment(KDL::Segment("arm_link_2", KDL::Joint(KDL::Joint::RotZ),
				frame_identity, KDL::RigidBodyInertia(1, frame_identity.Inverse() * KDL::Vector(1, 1, 1), KDL::RotationalInertia(1, 1, 1, 0, 0, 0))));
}

/**
 * Kinematic chain of the youbot manipulator
 * Args: chain
 * Output(s) - chainRobot (call-by-reference)
 **/
void KinematicChain::kinematicChainForYoubotManipulator(KDL::Chain &chainRobot)
{
	// Gravity vector declaration
    KDL::Vector grav(0.0, 0.0, -9.81);

	// Frame description for the individual body attached frames
	KDL::Frame  frame_identity = createTransform(0.000, 0.000, 0.000, 0.000, 0.000, 0.000),
				arm_joint_1_frame = createTransform(0.000, 0.000, M_PI, 0.024, 0.000, 0.115),
				arm_joint_2_frame = createTransform(-M_PI_2, 0.000, M_PI_2, 0.033, 0.000, 0.000),
				arm_joint_3_frame = createTransform(-M_PI_2, 0.000, 0.000, 0.155, 0.000, 0.000),
				arm_joint_4_frame = createTransform(0.000, 0.000, 0.000, 0.000, 0.135, 0.000),
				arm_joint_5_frame = createTransform(0.000, 0.000, -M_PI_2, 0.000, 0.1136, 0.000),
				arm_joint_gripper_frame = createTransform(M_PI, 0.000, 0.000, 0.000, 0.000, 0.05716);
	// CoM of the rigid bodies
	KDL::Vector com_body_1{0.01516, 0.00359,  0.03105},
				com_body_2{0.11397, 0.01500, -0.01903},
				com_body_3{0.00013, 0.10441,  0.02022},
				com_body_4{0.00015, 0.05353, -0.02464},
				com_body_5{0.00000, 0.00120, -0.01648};
	
	// Segment of the base
	chainRobot.addSegment(KDL::Segment("base_link", KDL::Joint(KDL::Joint::None), arm_joint_1_frame.Inverse(), 
									   KDL::RigidBodyInertia(constants::link_mass[0])));
	// Segment for the link 1
	chainRobot.addSegment(KDL::Segment("arm_link_1", KDL::Joint(KDL::Joint::RotZ),
					arm_joint_2_frame, KDL::RigidBodyInertia(constants::link_mass[1], arm_joint_2_frame.Inverse() * com_body_1)));
	// Segment for the link 2
	chainRobot.addSegment(KDL::Segment("arm_link_2", KDL::Joint(KDL::Joint::RotZ),
					arm_joint_3_frame, KDL::RigidBodyInertia(constants::link_mass[2], arm_joint_3_frame.Inverse() * com_body_2)));
	// Segment for the link 3
	chainRobot.addSegment(KDL::Segment("arm_link_3", KDL::Joint(KDL::Joint::RotZ),
					arm_joint_4_frame, KDL::RigidBodyInertia(constants::link_mass[3], arm_joint_4_frame.Inverse() * com_body_3)));
	// Segment for the link 4
	chainRobot.addSegment(KDL::Segment("arm_link_4", KDL::Joint(KDL::Joint::RotZ),
					arm_joint_5_frame, KDL::RigidBodyInertia(constants::link_mass[4], arm_joint_5_frame.Inverse() * com_body_4)));
	// Segment for the link 5
	chainRobot.addSegment(KDL::Segment("arm_link_5", KDL::Joint(KDL::Joint::RotZ, -1),
					arm_joint_gripper_frame, KDL::RigidBodyInertia(constants::link_mass[5], arm_joint_gripper_frame.Inverse() * com_body_5)));

	/*chainRobot.addSegment(KDL::Segment("arm_gripper", KDL::Joint(KDL::Joint::None),
                arm_joint_gripper_frame, KDL::RigidBodyInertia(constants::link_mass[6], armGripperInertialFrame.p, operator*(armGripperInertialFrame.M, 
                KDL::RotationalInertia(0.0002324, 0.0003629, 0.0002067, 0, 0, 0)))));*/

}
