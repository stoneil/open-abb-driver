#ifndef _ABB_TRAJECTORY_GENERATOR_H_
#define _ABB_TRAJECTORY_GENERATOR_H_

#include "argus_utils/geometry/PoseSE3.h"

#include <stdexcept>
#include <ros/ros.h>

#include "open_abb_driver/ABBKinematics.h"
#include "open_abb_driver/SimpleDijkstra.hpp"

namespace open_abb_driver
{
	
struct CartesianWaypoint
{
	argus::PoseSE3 pose;
	ros::Duration time;
};
typedef std::vector<CartesianWaypoint> CartesianTrajectory;

struct JointWaypoint
	: public DijkstraNode
{
	typedef std::shared_ptr<JointWaypoint> Ptr;
	
	JointAngles joints;
	ros::Duration time;
};
typedef std::vector<JointWaypoint> JointTrajectory;

class TrajectoryGenerator
{
public:
	
	typedef std::shared_ptr<TrajectoryGenerator> Ptr;
	
	TrajectoryGenerator();
	
	TrajectoryGenerator( const ABBKinematics::Ptr& kine );

	/*! \brief Attempt to generate a joint trajectory that matches the given Cartesian trajectory.
	 * May fail and throw a std::runtime_error if no IK solution is found. */
	JointTrajectory GenerateTrajectory( const JointAngles& init,
	                                    const CartesianTrajectory& reference );

private:
	
	ABBKinematics::Ptr _kinematics;
	
	/*! \brief Returns the cost of a trajectory, calculated as the sum of joint differences. */
	double CalculateTrajectoryCost( const JointTrajectory& traj );
	
};
	
}

#endif
