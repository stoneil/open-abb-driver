#ifndef _ABB_TRAJECTORY_GENERATOR_H_
#define _ABB_TRAJECTORY_GENERATOR_H_

#include "argus_utils/PoseSE3.h"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <stdexcept>

#include "open_abb_driver/ABBKinematics.h"
#include "open_abb_driver/SimpleDijkstra.hpp"

namespace open_abb_driver
{
	
	struct CartesianWaypoint
	{
		argus_utils::PoseSE3 pose;
		boost::posix_time::ptime time;
	};
	typedef std::vector<CartesianWaypoint> CartesianTrajectory;
	
	struct JointWaypoint
		: public DijkstraNode
	{
		typedef std::shared_ptr<JointWaypoint> Ptr;
		
		JointAngles joints;
		boost::posix_time::ptime time;
	};
	typedef std::vector<JointWaypoint> JointTrajectory;
	
	class TrajectoryGenerator
	{
	public:
		
		std::shared_ptr<TrajectoryGenerator> Ptr;
		
		TrajectoryGenerator();
		
		/*! \brief Attempt to generate a joint trajectory that matches the given Cartesian trajectory.
		 * May fail and throw a std::runtime_error if no IK solution is found. */
		JointTrajectory GenerateTrajectory( const CartesianTrajectory& reference );
	
	private:
		
		ABBKinematics kinematics;
		
		/*! \brief Returns the cost of a trajectory, calculated as the sum of joint differences. */
		double CalculateTrajectoryCost( const JointTrajectory& traj );
		
	};
	
}

#endif
