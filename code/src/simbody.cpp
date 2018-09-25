/*************************************************************************************
 * Copyright (C) 2018 Jeyaprakash Rajagopal <jeyaprakash.rajagopal@smail.inf.h-brs.de>
 * Version 1.0 
 * Modified Date: 15.07.2018
 * Filename 	: simbody.cpp
 * Path			: /src
 * Institute	: Hochschule Bonn-Rhein-Sieg University of Applied Sciences
 * Author 		: Sven Schneider, Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Prof. Dr. Roustiam Chakirov
 * Explanation 	: Source file for the simbody visualization
 *************************************************************************************/
#include "simbody.hpp"

#define KILOGRAM(x) (x)
#define KILOGRAM_SQUARE_METRE(x) (x)
#define DEGREE(x) (x)
#define METRE(x) (x)
#define NEWTON_METRE(x) (x)
#define NEWTON_METRE_PER_RADIAN(x) (x)
#define NEWTON_METRE_SECOND_PER_RADIAN(x) (x)
#define DEGREE_PER_SECOND(x) (x)
#define SECOND_PER_RADIAN(x) (x)

#define DEG_TO_RAD(x) (x) * M_PI / 180.0

/**
 * Constructor definition for the simbody simulator & visualizer : 
 **/
Simbody::Simbody( double cycle_time, bool visualize)
        : NUMBER_OF_BODIES(9), cycle_time_(cycle_time),
        SimTK::PeriodicEventReporter(cycle_time),
        matter_(*this), forces_(*this), integrator_(*this),
        time_stepper_(*this, integrator_)
{
	position_limit_.resize(5);
	position_limit_[0] = limit(-2.94961, 2.94961);
	position_limit_[1] = limit(-1.13446, 1.5708);
	position_limit_[2] = limit(-2.54818, 2.63545);
	position_limit_[3] = limit(-1.7889625, 1.7889625);
	position_limit_[4] = limit(-2.87979, 2.87979);
	
	velocity_limit_.resize(5);
	velocity_limit_[0] = limit(-1.5708, 1.5708);
	velocity_limit_[1] = limit(-1.5708, 1.5708);
	velocity_limit_[2] = limit(-1.5708, 1.5708);
	velocity_limit_[3] = limit(-1.5708, 1.5708);
	velocity_limit_[4] = limit(-1.5708, 1.5708);

	torque_limit_.resize(5);
	torque_limit_[0] = limit(-12.9012, 12.9012);
	torque_limit_[1] = limit(-12.9012, 12.9012);
	torque_limit_[2] = limit(-8.2700, 8.2700);
	torque_limit_[3] = limit(-4.1748, 4.1748);
	torque_limit_[4] = limit(-1.0000, 1.0000);
	
    measuredPositions_.resize(constants::MANIPULATOR_JNTS, 0.0);
    measuredVelocities_.resize(constants::MANIPULATOR_JNTS, 0.0);
    measuredAccelerations_.resize(constants::MANIPULATOR_JNTS, 0.0);
	measuredTorque_.resize(constants::MANIPULATOR_JNTS, 0.0);
    commanded_torque_.resize(constants::MANIPULATOR_JNTS);

    // Subscribe to sensor readings via the SimTK::PeriodicEventReporter
    // interface
    addEventReporter(this);

    // Specify where "top" is
    setUpDirection(-SimTK::ZAxis);

    // Simulated gravity
	gravity_ = SimTK::Force::Gravity(forces_, matter_, SimTK::UnitVec3(0, 0, -1.0), 9.81);

    // We want to provide custom forces for this mechanism via the
    // SimTK::Force::Custom::Implementation interface
    custom_force_ = SimTK::Force::Custom(forces_, this);

    // Don't show coordinate frames
    matter_.setShowDefaultGeometry(false);

    // Create the kinematic and dynamic model.
    createModel(matter_.Ground());

    // Set up visualization
    if (visualize) {
        visualizer_ = new SimTK::Visualizer(*this);
        visualizer_->setSystemUpDirection(SimTK::ZAxis);
        visualizer_->setCameraClippingPlanes(0.1, 1000.0);
        visualizer_->setShowSimTime(true);
        addEventReporter(new SimTK::Visualizer::Reporter(*visualizer_, 0.01));
    }

    // Initialize the system and state
    SimTK::MultibodySystem::realizeTopology();
    SimTK::State state = getDefaultState();
    // Simulate it
    time_stepper_.initialize(state);
}


/**
 * Destructor for the simbody simulator : 
 **/
Simbody::~Simbody()
{
    delete visualizer_;
}

/**
 * To return the acceleration of the manipulator joints : 
 * Args: NA
 * Output(s) - Measured acceleration vector
 **/
std::vector<double> Simbody::getJointAcceleration() const
{
    boost::mutex::scoped_lock lock(simulation_mutex_);
	return measuredAccelerations_;
}

/**
 * To compute the offset from the Kinematic to youBot coordinate frame : 
 * Args: Joint positions
 * Output(s) - Offset
 **/
void fromSimbodyToKdl(std::vector<double> &jntpos)
{
	std::vector<double> jointpositions(constants::MANIPULATOR_JNTS, 0.0);
	for(int index = 0; index < constants::MANIPULATOR_JNTS; index++)
	{
	 	jointpositions[index] = (jntpos[index] + constants::youbotOffset[index]);
	}
	
	jntpos = jointpositions;
}

/**
 * To return the position of the manipulator joints : 
 * Args: NA
 * Output(s) - Measured joint position vector
 **/
std::vector<double> Simbody::getJointPosition() const
{
    boost::mutex::scoped_lock lock(simulation_mutex_);
	
	std::vector<double> measuredpositions(constants::MANIPULATOR_JNTS, 0.0);
	measuredpositions = measuredPositions_;
	fromSimbodyToKdl(measuredpositions);

    return measuredpositions;
}

/**
 * To return the velocity of the manipulator joints : 
 * Args: NA
 * Output(s) - Measured velocity vector
 **/
std::vector<double> Simbody::getJointVelocity() const
{
    boost::mutex::scoped_lock lock(simulation_mutex_);

    return measuredVelocities_;
}

/**
 * To set the joint positions to the robot joints. 
 */
void Simbody::setJointPosition(std::vector<double> jointPositions) const 
{
}
		
/**
 * To command the torque to the manipulator joints : 
 * Args: NA
 **/
bool Simbody::setJointTorque(const std::vector<double> &torque)
{
    assert(torque.size() == constants::MANIPULATOR_JNTS);
    //boost::mutex::scoped_lock lock(simulation_mutex_);

    for (int i = 0; i < constants::MANIPULATOR_JNTS; i++) {
        if (torque[i] < torque_limit_[i].lower) {
            std::cout << "Warning: Clamping torque to lower limit" << std::endl;
            commanded_torque_[i] = torque_limit_[i].lower;
        } else if (torque[i] > torque_limit_[i].upper) {
            std::cout << "Warning: Clamping torque to upper limit" << std::endl;
            commanded_torque_[i] = torque_limit_[i].upper;
        } else {
            commanded_torque_[i] = torque[i];
        }
    }
    return true;
}

void Simbody::setJointVelocity(const std::vector<double> &velocity)
{
	
}

/**
 * To return the clammped joint torques before commanding the torques to the manipulator joints : 
 * Args: NA
 * Output(s) - clampped joint torque
 **/
 std::vector<double> Simbody::clampJointTorque(
        const std::vector<double> &torque) const
{
    assert(torque.size() == constants::MANIPULATOR_JNTS);

    std::vector<double> clamped(constants::MANIPULATOR_JNTS);
    for (int i = 0; i < constants::MANIPULATOR_JNTS; i++) {
        if (torque[i] < torque_limit_[i].lower) {
            clamped[i] = torque_limit_[i].lower;
        } else if (torque[i] > torque_limit_[i].upper) {
            clamped[i] = torque_limit_[i].upper;
        } else {
            clamped[i] = torque[i];
        }
    }

    return clamped;
}

/**
 * To return the torque from the manipulator joints : 
 * Args: NA
 * Output(s) - Measured torque
 **/
 std::vector<double> Simbody::getJointTorque() const {
    boost::mutex::scoped_lock lock(simulation_mutex_);
	std::cout << "Simbody Torques: " << std::endl;
	for(int index = 0; index < constants::MANIPULATOR_JNTS; index++)
		std::cout << index << " " << measuredTorque_[index] << std::endl;

	return measuredTorque_;
}

void Simbody::initialize()
{
	// Start the simulation in a thread
	update();
}


void Simbody::update()
{
	time_stepper_.stepTo(time_stepper_.getTime() + cycle_time_);
}

/**
 * Invoked by the event reporter whenever an event occurs: 
 * Args: Current state of the system
 * Output(s) - NA
 **/
void Simbody::handleEvent(const SimTK::State &state) const
{
    boost::mutex::scoped_lock lock(simulation_mutex_);
	
	// In velocity stage
	// Getting the joint position from all the joints of the manipulator
	const SimTK::Vector &jointPositions = state.getQ();
	for (unsigned int index = 0; index < jointPositions.size(); index++) {
        if (jointPositions[index] > position_limit_[index].upper) std::cout << "Position in joint " << index + 1 << " higher than limit: " << jointPositions[index] << " rad > " << position_limit_[index].upper << " rad" << std::endl;
        if (jointPositions[index] < position_limit_[index].lower) std::cout << "Position in joint " << index + 1 << " lower than limit: " << jointPositions[index] << " rad < " << position_limit_[index].lower << " rad" << std::endl;
		
		measuredPositions_[index] = jointPositions[index];
		//std::cout << "Pos : " << measuredPositions_[index] << std::endl;
	}
	// Getting the joint velocity from all the joints of the manipulator
	const SimTK::Vector &jointVelocities = state.getQDot();
	for (unsigned int index = 0; index < jointVelocities.size(); index++) {
        if (jointVelocities[index] > velocity_limit_[index].upper) std::cout << "Velocity in joint " << index + 1 << " higher than limit: " << jointVelocities[index] << " rad/s > " << velocity_limit_[index].upper << " rad/s" << std::endl;
        if (jointVelocities[index] < velocity_limit_[index].lower) std::cout << "Velocity in joint " << index + 1 << " lower than limit: " << jointVelocities[index] << " rad/s < " << velocity_limit_[index].lower << " rad/s" << std::endl;
		measuredVelocities_[index] = jointVelocities[index];
		//std::cout << "Vel : " << measuredVelocities_[index] << std::endl;
	}
	
	// Realizing the acceleration stage in order to get the acceleration from the joints
	SimTK::MultibodySystem::realize(state, SimTK::Stage::Acceleration);
	const SimTK::Vector &jointAccelerations = state.getQDotDot();
	for (unsigned int index = 0; index < jointAccelerations.size(); index++) {
		measuredAccelerations_[index] = jointAccelerations[index];
		//std::cout << "Acc : " << measuredAccelerations_[index] << std::endl;
	}
	//std::cout << state.getSystemStage() << std::endl;
    SimTK::Vector torque_inertia(constants::MANIPULATOR_JNTS);
    const SimTK::Vector_<SimTK::SpatialVec> &force_inertia = gravity_.getBodyForces(state);

    // Calculate centrifugal forces (Coriolis and gyroscopic forces)
    SimTK::Vector_<SimTK::SpatialVec> force_bias = SimTK::Vector_<SimTK::SpatialVec>(matter_.getNumBodies());
    for (int i = 0; i < matter_.getNumBodies(); i++) {
        force_bias[i] = matter_.getTotalCentrifugalForces(state, SimTK::MobilizedBodyIndex(i));
    }
    // Map Cartesian forces to joint torques
    SimTK::Vector torque(constants::MANIPULATOR_JNTS);
    matter_.multiplyBySystemJacobianTranspose(state, -force_inertia - force_bias, torque);
	
    for(int index = 0; index < torque.size(); index++) {
		measuredTorque_[index] = torque[index];
	}
}

/**
 * To calculate the force for the given state : 
 * Args: state
 * Output(s) - NA
 **/
void Simbody::calcForce(const SimTK::State &state,
        SimTK::Vector_<SimTK::SpatialVec> &bodyForces,
        SimTK::Vector_<SimTK::Vec3> &particleForces,
        SimTK::Vector &mobilityForces) const
{	
    // Compute gravity and bias force compensation torque (via inverse dynamics)
    SimTK::Vector compensation_torque;
    matter_.calcResidualForceIgnoringConstraints(state, mobilityForces,
            bodyForces, SimTK::Vector(), compensation_torque);

    for (int i = 0; i < constants::MANIPULATOR_JNTS; i++) {
     /*   // When the joint is (almost) not moving, we need to consider static
        // friction
        
        if (fabs(state.getU()[i]) <= 0.0001) {
            // All the torque acting on the joint. It consists of the
            // contribution of the commanded torque and the torque due to
            // gravity and bias force.
            double applied_torque
                    = commanded_torque_[i] - compensation_torque[i];

            if (fabs(applied_torque) < break_away_friction_[i]) {
                // If all the applied torque is less than the joint's break-away
                // torque (aka friction loss), the joint should remain at
                // stand-still. Thus, only compensating torques are commanded.
                mobilityForces[i] = compensation_torque[i];
            } else {
                // For a brief instant when the joint is still not moving, but
                // the applied torque crosses the break-away torque, the
                // commanded torque will be reduced by the break-away torque.
                // For the next instant in time, the joint be moving.
                mobilityForces[i] = commanded_torque_[i] - break_away_friction_[i];
            }
        } else {*/
        
            // For the moving joint only the commanded torque is applied.
            mobilityForces[i] = commanded_torque_[i];
       // }
    }
}


SimTK::Real Simbody::calcPotentialEnergy(const SimTK::State &state) const
{
    return 0;
}


SimTK::Transform Simbody::createTransform(
        double origin_x, double origin_y, double origin_z,
        double gamma, double beta, double alpha)
{
    // The Simbody specification uses extrinsic Tait-Bryan angles with the ZYX
    // sequence
    double ca = cos(alpha);
    double sa = sin(alpha);
    double cb = cos(beta);
    double sb = sin(beta);
    double cc = cos(gamma);
    double sc = sin(gamma);

    return SimTK::Transform(
            SimTK::Rotation(SimTK::Mat33(
                            cb * cc    ,           -cb * sc     ,     sb  ,
                 sa * sb * cc + ca * sc, -sa * sb * sc + ca * cc, -sa * cb,
                -ca * sb * cc + sa * sc,  ca * sb * sc + sa * cc,  ca * cb)),
            SimTK::Vec3(origin_x, origin_y, origin_z));
}


void Simbody::createModel(SimTK::MobilizedBody &ground)
{
    //
    // Body parameters
    //
    std::vector<SimTK::Transform> pose_joint_attachment(NUMBER_OF_BODIES);
    pose_joint_attachment[ID_BODY_BASE        ] = createTransform(METRE(0.0),   METRE( 0.0),    METRE(0.0),     DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(   0.0)));
    pose_joint_attachment[ID_BODY_LINK_1      ] = createTransform(METRE(0.024), METRE( 0.0),    METRE(0.115),   DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE( 180.0)));
    pose_joint_attachment[ID_BODY_LINK_2      ] = createTransform(METRE(0.033), METRE( 0.0),    METRE(0.0),     DEG_TO_RAD(DEGREE(- 90.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(  90.0)));
    pose_joint_attachment[ID_BODY_LINK_3      ] = createTransform(METRE(0.155), METRE( 0.0),    METRE(0.0),     DEG_TO_RAD(DEGREE(- 90.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(   0.0)));
    pose_joint_attachment[ID_BODY_LINK_4      ] = createTransform(METRE(0.0),   METRE( 0.135),  METRE(0.0),     DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(   0.0)));
    pose_joint_attachment[ID_BODY_LINK_5      ] = createTransform(METRE(0.0),   METRE( 0.1136), METRE(0.0),     DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(- 90.0)));
    pose_joint_attachment[ID_BODY_GRIPPER_BASE] = createTransform(METRE(0.0),   METRE( 0.0),    METRE(0.05716), DEG_TO_RAD(DEGREE( 180.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(   0.0)));
    pose_joint_attachment[ID_BODY_LEFT_FINGER ] = createTransform(METRE(0.0),   METRE( 0.009),  METRE(0.0),     DEG_TO_RAD(DEGREE(  90.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(   0.0)));
    pose_joint_attachment[ID_BODY_RIGHT_FINGER] = createTransform(METRE(0.0),   METRE(-0.009),  METRE(0.0),     DEG_TO_RAD(DEGREE(- 90.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(   0.0)));

    std::vector<SimTK::Transform> pose_inertia(NUMBER_OF_BODIES);
    pose_inertia[ID_BODY_BASE        ] = createTransform(METRE(0.0),     METRE(0.0),     METRE( 0.0),     DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(  0.0)), DEG_TO_RAD(DEGREE(  0.0)));
    pose_inertia[ID_BODY_LINK_1      ] = createTransform(METRE(0.01516), METRE(0.00359), METRE( 0.03105), DEG_TO_RAD(DEGREE( 180.0)), DEG_TO_RAD(DEGREE( 20.0)), DEG_TO_RAD(DEGREE(  0.0)));
    pose_inertia[ID_BODY_LINK_2      ] = createTransform(METRE(0.11397), METRE(0.015),   METRE(-0.01903), DEG_TO_RAD(DEGREE(- 90.0)), DEG_TO_RAD(DEGREE(  0.0)), DEG_TO_RAD(DEGREE(-90.0)));
    pose_inertia[ID_BODY_LINK_3      ] = createTransform(METRE(0.00013), METRE(0.10441), METRE( 0.02022), DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(  0.0)), DEG_TO_RAD(DEGREE( 90.0)));
    pose_inertia[ID_BODY_LINK_4      ] = createTransform(METRE(0.00015), METRE(0.05353), METRE(-0.02464), DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(180.0)), DEG_TO_RAD(DEGREE(  0.0)));
    pose_inertia[ID_BODY_LINK_5      ] = createTransform(METRE(0.0),     METRE(0.0012),  METRE(-0.01648), DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE( 90.0)), DEG_TO_RAD(DEGREE(  0.0)));
    pose_inertia[ID_BODY_GRIPPER_BASE] = createTransform(METRE(0.0),     METRE(0.0),     METRE( 0.0289),  DEG_TO_RAD(DEGREE( 180.0)), DEG_TO_RAD(DEGREE(  0.0)), DEG_TO_RAD(DEGREE( 90.0)));
    pose_inertia[ID_BODY_LEFT_FINGER ] = createTransform(METRE(0.0),     METRE(0.0),     METRE( 0.0),     DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(  0.0)), DEG_TO_RAD(DEGREE(  0.0)));
    pose_inertia[ID_BODY_RIGHT_FINGER] = createTransform(METRE(0.0),     METRE(0.0),     METRE( 0.0),     DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(  0.0)), DEG_TO_RAD(DEGREE(  0.0)));    

    std::vector<SimTK::Transform> pose_decoration(NUMBER_OF_BODIES);
    pose_decoration[ID_BODY_LEFT_FINGER ] = SimTK::Transform(pose_joint_attachment[ID_BODY_LEFT_FINGER ].R().invert());
    pose_decoration[ID_BODY_RIGHT_FINGER] = SimTK::Transform(pose_joint_attachment[ID_BODY_RIGHT_FINGER].R().invert());

    std::vector<SimTK::Inertia> inertia_central(NUMBER_OF_BODIES);
    inertia_central[ID_BODY_BASE        ] = SimTK::Inertia(KILOGRAM_SQUARE_METRE(0.00001),    KILOGRAM_SQUARE_METRE(0.00001),    KILOGRAM_SQUARE_METRE(0.00001));
    inertia_central[ID_BODY_LINK_1      ] = SimTK::Inertia(KILOGRAM_SQUARE_METRE(0.0029525),  KILOGRAM_SQUARE_METRE(0.0060091),  KILOGRAM_SQUARE_METRE(0.0058821));
    inertia_central[ID_BODY_LINK_2      ] = SimTK::Inertia(KILOGRAM_SQUARE_METRE(0.0031145),  KILOGRAM_SQUARE_METRE(0.0005843),  KILOGRAM_SQUARE_METRE(0.0031631));
    inertia_central[ID_BODY_LINK_3      ] = SimTK::Inertia(KILOGRAM_SQUARE_METRE(0.00172767), KILOGRAM_SQUARE_METRE(0.00041967), KILOGRAM_SQUARE_METRE(0.00184680));
    inertia_central[ID_BODY_LINK_4      ] = SimTK::Inertia(KILOGRAM_SQUARE_METRE(0.0006764),  KILOGRAM_SQUARE_METRE(0.0010573),  KILOGRAM_SQUARE_METRE(0.0006610));
    inertia_central[ID_BODY_LINK_5      ] = SimTK::Inertia(KILOGRAM_SQUARE_METRE(0.0001934),  KILOGRAM_SQUARE_METRE(0.0001602),  KILOGRAM_SQUARE_METRE(0.0000689));
    inertia_central[ID_BODY_GRIPPER_BASE] = SimTK::Inertia(KILOGRAM_SQUARE_METRE(0.0002324),  KILOGRAM_SQUARE_METRE(0.0003629),  KILOGRAM_SQUARE_METRE(0.0002067));
    inertia_central[ID_BODY_LEFT_FINGER ] = SimTK::Inertia(KILOGRAM_SQUARE_METRE(0.00001),    KILOGRAM_SQUARE_METRE(0.00001),    KILOGRAM_SQUARE_METRE(0.00001));
    inertia_central[ID_BODY_RIGHT_FINGER] = SimTK::Inertia(KILOGRAM_SQUARE_METRE(0.00001),    KILOGRAM_SQUARE_METRE(0.00001),    KILOGRAM_SQUARE_METRE(0.00001));

    std::vector<double> mass(NUMBER_OF_BODIES); // [kg]
    mass[ID_BODY_BASE        ] = KILOGRAM(0.961);
    mass[ID_BODY_LINK_1      ] = KILOGRAM(1.390);
    mass[ID_BODY_LINK_2      ] = KILOGRAM(1.318);
    mass[ID_BODY_LINK_3      ] = KILOGRAM(0.821);
    mass[ID_BODY_LINK_4      ] = KILOGRAM(0.769);
    mass[ID_BODY_LINK_5      ] = KILOGRAM(0.687);
    mass[ID_BODY_GRIPPER_BASE] = KILOGRAM(0.199);
    mass[ID_BODY_LEFT_FINGER ] = KILOGRAM(0.01);
    mass[ID_BODY_RIGHT_FINGER] = KILOGRAM(0.01);
	
    std::vector<std::string> mesh_name(NUMBER_OF_BODIES);
    mesh_name[ID_BODY_BASE        ] = "/home/jp/computed_torque_control/geometry/arm_base_frame.stl";
    mesh_name[ID_BODY_LINK_1      ] = "/home/jp/computed_torque_control/geometry/arm_joint_1.stl";
    mesh_name[ID_BODY_LINK_2      ] = "/home/jp/computed_torque_control/geometry/arm_joint_2.stl";
    mesh_name[ID_BODY_LINK_3      ] = "/home/jp/computed_torque_control/geometry/arm_joint_3.stl";
    mesh_name[ID_BODY_LINK_4      ] = "/home/jp/computed_torque_control/geometry/arm_joint_4.stl";
    mesh_name[ID_BODY_LINK_5      ] = "/home/jp/computed_torque_control/geometry/arm_joint_5.stl";
    mesh_name[ID_BODY_GRIPPER_BASE] = "/home/jp/computed_torque_control/geometry/gripper_base_frame.stl";
    mesh_name[ID_BODY_LEFT_FINGER ] = "/home/jp/computed_torque_control/geometry/gripper_left_finger.stl";
    mesh_name[ID_BODY_RIGHT_FINGER] = "/home/jp/computed_torque_control/geometry/gripper_right_finger.stl";

    std::vector<SimTK::Vec3> mesh_color(NUMBER_OF_BODIES);
    mesh_color[ID_BODY_BASE        ] = SimTK::Black;
    mesh_color[ID_BODY_LINK_1      ] = SimTK::Orange;
    mesh_color[ID_BODY_LINK_2      ] = SimTK::Orange;
    mesh_color[ID_BODY_LINK_3      ] = SimTK::Orange;
    mesh_color[ID_BODY_LINK_4      ] = SimTK::Orange;
    mesh_color[ID_BODY_LINK_5      ] = SimTK::Orange;
    mesh_color[ID_BODY_GRIPPER_BASE] = SimTK::Black;
    mesh_color[ID_BODY_LEFT_FINGER ] = SimTK::Black;
    mesh_color[ID_BODY_RIGHT_FINGER] = SimTK::Black;

    //
    // Joint parameters
    //
    std::vector<limit> position_limit(constants::MANIPULATOR_JNTS); // [rad]
    position_limit[ID_JOINT_1] = limit(DEG_TO_RAD(DEGREE(-169.0)), DEG_TO_RAD(DEGREE(169.0)));
    position_limit[ID_JOINT_2] = limit(DEG_TO_RAD(DEGREE(- 65.0)), DEG_TO_RAD(DEGREE( 90.0)));
    position_limit[ID_JOINT_3] = limit(DEG_TO_RAD(DEGREE(-151.0)), DEG_TO_RAD(DEGREE(146.0)));
    position_limit[ID_JOINT_4] = limit(DEG_TO_RAD(DEGREE(-102.5)), DEG_TO_RAD(DEGREE(102.5)));
    position_limit[ID_JOINT_5] = limit(DEG_TO_RAD(DEGREE(-165.0)), DEG_TO_RAD(DEGREE(165.0)));

    std::vector<limit> velocity_limit(constants::MANIPULATOR_JNTS); // [rad/s]
    velocity_limit[ID_JOINT_1] = limit(DEG_TO_RAD(DEGREE_PER_SECOND(-90.0)), DEG_TO_RAD(DEGREE_PER_SECOND(90.0)));
    velocity_limit[ID_JOINT_2] = limit(DEG_TO_RAD(DEGREE_PER_SECOND(-90.0)), DEG_TO_RAD(DEGREE_PER_SECOND(90.0)));
    velocity_limit[ID_JOINT_3] = limit(DEG_TO_RAD(DEGREE_PER_SECOND(-90.0)), DEG_TO_RAD(DEGREE_PER_SECOND(90.0)));
    velocity_limit[ID_JOINT_4] = limit(DEG_TO_RAD(DEGREE_PER_SECOND(-90.0)), DEG_TO_RAD(DEGREE_PER_SECOND(90.0)));
    velocity_limit[ID_JOINT_5] = limit(DEG_TO_RAD(DEGREE_PER_SECOND(-90.0)), DEG_TO_RAD(DEGREE_PER_SECOND(90.0)));

    torque_limit_.resize(constants::MANIPULATOR_JNTS); // [Nm]
    torque_limit_[ID_JOINT_1] = limit(NEWTON_METRE(-9.5), NEWTON_METRE(9.5));
    torque_limit_[ID_JOINT_2] = limit(NEWTON_METRE(-9.5), NEWTON_METRE(9.5));
    torque_limit_[ID_JOINT_3] = limit(NEWTON_METRE(-6.0), NEWTON_METRE(6.0));
    torque_limit_[ID_JOINT_4] = limit(NEWTON_METRE(-2.0), NEWTON_METRE(2.0));
    torque_limit_[ID_JOINT_5] = limit(NEWTON_METRE(-1.0), NEWTON_METRE(1.0));

    std::vector<double> viscous_friction(constants::MANIPULATOR_JNTS); // [Nms/rad]
    viscous_friction[ID_JOINT_1] = NEWTON_METRE_SECOND_PER_RADIAN(0.1);
    viscous_friction[ID_JOINT_2] = NEWTON_METRE_SECOND_PER_RADIAN(0.1);
    viscous_friction[ID_JOINT_3] = NEWTON_METRE_SECOND_PER_RADIAN(0.1);
    viscous_friction[ID_JOINT_4] = NEWTON_METRE_SECOND_PER_RADIAN(0.1);
    viscous_friction[ID_JOINT_5] = NEWTON_METRE_SECOND_PER_RADIAN(0.1);

    break_away_friction_.resize(constants::MANIPULATOR_JNTS); // [Nm]
    break_away_friction_[ID_JOINT_1] = NEWTON_METRE(1.0);
    break_away_friction_[ID_JOINT_2] = NEWTON_METRE(1.0);
    break_away_friction_[ID_JOINT_3] = NEWTON_METRE(1.0);
    break_away_friction_[ID_JOINT_4] = NEWTON_METRE(1.0);
    break_away_friction_[ID_JOINT_5] = NEWTON_METRE(1.0);

    std::vector<double> position_limit_stiffness(constants::MANIPULATOR_JNTS); // [Nm/rad]
    position_limit_stiffness[ID_JOINT_1] = NEWTON_METRE_PER_RADIAN(10000.0);
    position_limit_stiffness[ID_JOINT_2] = NEWTON_METRE_PER_RADIAN(10000.0);
    position_limit_stiffness[ID_JOINT_3] = NEWTON_METRE_PER_RADIAN(10000.0);
    position_limit_stiffness[ID_JOINT_4] = NEWTON_METRE_PER_RADIAN(10000.0);
    position_limit_stiffness[ID_JOINT_5] = NEWTON_METRE_PER_RADIAN(10000.0);

    std::vector<double> position_limit_dissipation(constants::MANIPULATOR_JNTS); // [s/rad]
    position_limit_dissipation[ID_JOINT_1] = SECOND_PER_RADIAN(3.0);
    position_limit_dissipation[ID_JOINT_2] = SECOND_PER_RADIAN(3.0);
    position_limit_dissipation[ID_JOINT_3] = SECOND_PER_RADIAN(3.0);
    position_limit_dissipation[ID_JOINT_4] = SECOND_PER_RADIAN(3.0);
    position_limit_dissipation[ID_JOINT_5] = SECOND_PER_RADIAN(3.0);

    // Note on Simbody inertia
    // ======================
    // The Simbody specification provides:
    // 1. the principal moments of inertia (i.e. in the "CoM frame" which is
    //    originated at the CoM and oriented along the principal axes)
    // 2. the pose of the "CoM frame" w.r.t. the joint frame
    // Simbody requires the inertia to be expressed w.r.t. the body's reference
    // frame. Note that in the following kinematic chain, the body's reference
    // frame coincides with the body's joint attachment frame (to the parent),
    // i.e. the second transformation in the joint description is an identity
    // transformation. Because of this convention, we need to re-express and
    // shift the inertia with the inverse of the Simbody specification.


    // Create bodies
    std::vector<SimTK::Body> rigid_body(NUMBER_OF_BODIES);

    for (int i = 0; i < NUMBER_OF_BODIES; i++) {
        SimTK::Inertia inertia_root = inertia_central[i]
            .reexpress(pose_inertia[i].R().invert())
            .shiftFromMassCenter(-pose_inertia[i].p(), mass[i]);

        SimTK::PolygonalMesh mesh;
        mesh.loadStlFile(mesh_name[i]);

        SimTK::DecorativeMesh geometry(mesh);
        geometry.setColor(mesh_color[i]);

        rigid_body[i] = SimTK::Body::Rigid(
            SimTK::MassProperties(mass[i], pose_inertia[i].p(), inertia_root));
        rigid_body[i].addDecoration(pose_decoration[i], geometry);
    }

    // Create joints
    std::vector<SimTK::MobilizedBody> body(NUMBER_OF_BODIES);

    // Ground to robot base
    int body_index = 0;
    body[body_index] = SimTK::MobilizedBody::Weld(
        ground,
        SimTK::Transform(SimTK::Vec3(0.0, 0.0, 0.3)),
        rigid_body[body_index],
        SimTK::Transform(SimTK::Vec3(0.0, 0.0, 0.0)));
    body_index++;
    
	SimTK::MobilizedBody::Direction direction = SimTK::MobilizedBody::Direction::Forward;
    const int INDEX_OFFSET = 1; // offset between joint and body indices
    for (; body_index < INDEX_OFFSET + constants::MANIPULATOR_JNTS; body_index++) {
        int joint_index = body_index - INDEX_OFFSET;
		// joint 5 is reversed with the direction
		if(joint_index == ID_JOINT_5) {
			direction = SimTK::MobilizedBody::Direction::Reverse;
		}
		
        body[body_index] = SimTK::MobilizedBody::Pin(
            body[body_index - 1],
            pose_joint_attachment[body_index],
            rigid_body[body_index],
            SimTK::Transform(SimTK::Vec3(0.0, 0.0, 0.0)), direction);

        // Joint position limit
        /*SimTK::Force::MobilityLinearStop(
            forces_, body[body_index], SimTK::MobilizerQIndex(0),
            position_limit_stiffness[joint_index],
            position_limit_dissipation[joint_index],
            position_limit[joint_index].lower,
            position_limit[joint_index].upper);

        // Viscous friction
        SimTK::Force::MobilityLinearDamper(
            forces_, body[body_index], SimTK::MobilizerQIndex(0),
            viscous_friction[joint_index]);*/
    }

    // Arm to gripper base
/*    body[body_index] = SimTK::MobilizedBody::Weld(
        body[body_index - 1],
        pose_joint_attachment[body_index],
        rigid_body[body_index],
        SimTK::Transform(SimTK::Vec3(0.0, 0.0, 0.0)));
    body_index++;

    // Left finger
    body[body_index] = SimTK::MobilizedBody::Weld(
        body[body_index - 1],    // gripper base
        pose_joint_attachment[body_index],
        rigid_body[body_index],
        SimTK::Transform(SimTK::Vec3(0.0, 0.0, 0.0)));
    body_index++;

    // Right finger
    body[body_index] = SimTK::MobilizedBody::Weld(
        body[body_index - 2],  // gripper base
        pose_joint_attachment[body_index],
        rigid_body[body_index],
        SimTK::Transform(SimTK::Vec3(0.0, 0.0, 0.0)));
    body_index++;
*/

/*
    // Left finger
    body[body_index] = SimTK::MobilizedBody::Slider(
        body[body_index - 1],    // gripper base
        pose_joint_attachment[body_index],
        rigid_body[body_index],
        SimTK::Transform(SimTK::Vec3(0.0, 0.0, 0.0)));
    SimTK::Force::MobilityLinearStop(
        forces_, body[body_index], SimTK::MobilizerQIndex(0),
        NEWTON_METRE_PER_RADIAN(10000), SECOND_PER_RADIAN(3.0),
        METRE(0.0), METRE(0.0125));
    SimTK::Force::MobilityLinearDamper(
        forces_, body[body_index], SimTK::MobilizerQIndex(0),
        NEWTON_METRE_SECOND_PER_RADIAN(0.1));
    body_index++;

    // Right finger
    body[body_index] = SimTK::MobilizedBody::Slider(
        body[body_index - 2],  // gripper base
        pose_joint_attachment[body_index],
        rigid_body[body_index],
        SimTK::Transform(SimTK::Vec3(0.0, 0.0, 0.0)));
    SimTK::Force::MobilityLinearStop(
        forces_, body[body_index], SimTK::MobilizerQIndex(0),
        NEWTON_METRE_PER_RADIAN(10000), SECOND_PER_RADIAN(3.0),
        METRE(0.0), METRE(0.0125));
    SimTK::Force::MobilityLinearDamper(
        forces_, body[body_index], SimTK::MobilizerQIndex(0),
        NEWTON_METRE_SECOND_PER_RADIAN(0.1));
    body_index++;
*/
}

