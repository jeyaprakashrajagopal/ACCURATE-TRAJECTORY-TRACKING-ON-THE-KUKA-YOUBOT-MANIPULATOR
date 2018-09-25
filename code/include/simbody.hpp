/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Modified Date: 15.07.2018
 * Filename 	: simbody.hpp
 * Path			: /include
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Sven Schneider, Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov
 * Explanation 	: Header file for the simbody visualization
 *************************************************************************************/
#ifndef __SIMBODY__
#define __SIMBODY__

#include <vector>

#include <Simbody.h>
#include "youbot_hal.hpp"
#include "constants.hpp"
#include <boost/thread/thread.hpp>

class Simbody : public YoubotHal, SimTK::MultibodySystem, SimTK::PeriodicEventReporter,
        SimTK::Force::Custom::Implementation
{
    public:
        /**
         * @param cycle_time The duration of one cycle (simulation and sensor
         *                   reading) in [s].
         * @param visualize When set to 'true' visualize the robot's simulation.
         */
        Simbody(double cycle_time = 0.001, bool visualize = false);

        /**
         * Dtor.
         */
        virtual ~Simbody();

        /**
         * Read the current position of the robot's joints.
         */
        std::vector<double> getJointPosition() const;

        /**
         * Read the current velocity of the robot's joints.
         */
        std::vector<double> getJointVelocity() const;
		
		std::vector<double> getJointTorque() const;
		
 		std::vector<double> getJointAcceleration() const;
        /**
         * Start the simulation.
         *
         * This function will start a thread for the simulation and immediately
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
        /**
         * Clamp all the torques in the provided vector to the permissible range
         * of joint torques for this robot.
         */
        std::vector<double> clampJointTorque(
                const std::vector<double> &torque) const;

		/**
         * To set the joint positions to the robot joints. 
         */
		void setJointPosition(std::vector<double> jointPositions) const;
		
    private:
        /**
         * Forbid copy construction.
         */
        Simbody(const Simbody &);

        /**
         * Forbid assignment.
         */
        Simbody &operator=(const Simbody &);

        /**
         * Given the parameters of the transformation, create a representation
         * that is compatible with Simbody.
         */
        SimTK::Transform createTransform(
                double origin_x, double origin_y, double origin_z,
                double gamma, double beta, double alpha);

        /**
         * Create the overall model of the robot.
         */
        void createModel(SimTK::MobilizedBody &ground);

        /**
         * The main loop that is running in the simulation thread. It advances
         * the simulation.
         */
        void update();

        /**
         * This method is called whenever an event occurs.
         * @see SimTK::PeriodicEventReporter::handleEvent
         */
        void handleEvent(const SimTK::State &state) const;

        /**
         * @see SimTK::Force::Custom::Implementation
         */
        void calcForce(const SimTK::State &state,
                SimTK::Vector_<SimTK::SpatialVec> &bodyForces,
                SimTK::Vector_<SimTK::Vec3> &particleForces,
                SimTK::Vector &mobilityForces) const;

        /**
         * @see SimTK::Force::Custom::Implementation
         */
        SimTK::Real calcPotentialEnergy(const SimTK::State &state) const;
        
        struct limit
        {
            limit() : lower(0.0), upper(0.0) {}
            limit(double l, double u) : lower(l), upper(u) {}

            double lower;
            double upper;
        };

        enum body_id
        {
            ID_BODY_BASE         = 0,
            ID_BODY_LINK_1       = 1,
            ID_BODY_LINK_2       = 2,
            ID_BODY_LINK_3       = 3,
            ID_BODY_LINK_4       = 4,
            ID_BODY_LINK_5       = 5,
            ID_BODY_GRIPPER_BASE = 6,
            ID_BODY_LEFT_FINGER  = 7,
            ID_BODY_RIGHT_FINGER = 8
        };

        enum joint_id
        {
            ID_JOINT_1 = 0,
            ID_JOINT_2 = 1,
            ID_JOINT_3 = 2,
            ID_JOINT_4 = 3,
            ID_JOINT_5 = 4
        };

        const int NUMBER_OF_BODIES;

        std::vector<limit> position_limit_;             // [rad]
        std::vector<limit> velocity_limit_;             // [rad/s]
        std::vector<limit> torque_limit_;               // [Nm]
        mutable std::vector<double> measuredPositions_; // [rad]
        mutable std::vector<double> measuredVelocities_; // [rad/s]
		mutable std::vector<double> measuredAccelerations_;
		mutable std::vector<double> measuredTorque_;
        
        SimTK::Vector commanded_torque_;                // [Nm]
        std::vector<double> break_away_friction_;       // [Nm]

        double cycle_time_;                             // [s]

        SimTK::SimbodyMatterSubsystem matter_;
        SimTK::GeneralForceSubsystem forces_;
        SimTK::Force::Gravity gravity_;
        SimTK::Force::Custom custom_force_;
        SimTK::Visualizer *visualizer_;
        SimTK::RungeKuttaMersonIntegrator integrator_;
        SimTK::TimeStepper time_stepper_;
		boost::thread simulation_thread_;
        mutable boost::mutex simulation_mutex_;
};
#endif
