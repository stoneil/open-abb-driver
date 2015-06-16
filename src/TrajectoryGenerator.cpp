#include "open_abb_driver/TrajectoryGenerator.h"

namespace open_abb_driver
{
	
	TrajectoryGenerator::TrajectoryGenerator() {}
	
	JointTrajectory TrajectoryGenerator::GenerateTrajectory( const CartesianTrajectory& reference )
	{
		std::vector<JointAngles> csol;
		std::vector<JointWaypoint::Ptr> prevSolutions, currSolutions, solutions;
		DijkstraSearch search;
		
		for( int i = 0; i < reference.size(); i++ )
		{
			if( !kinematics.ComputeIK( reference[i].pose, csol ) || csol.size() == 0 )
			{
				std::stringstream ss;
				ss << "Waypoint " << i << " of " << reference[i].pose << " is infeasible.";
				throw std::runtime_error( ss.str() );
			}
			
			currSolutions.clear();
			BOOST_FOREACH( const JointAngles& sol, csol )
			{
				JointWaypoint::Ptr cs = std::make_shared<JointWaypoint>();
				cs->joints = sol;
				cs->time = reference[i].time;
				currSolutions.push_back( cs );
				
				if( i == 0 )
				{
					std::cout << "Adding initial node: " << cs->joints << std::endl;
					search.AddNode( cs, true, false );
				}
				else
				{
					search.AddNode( cs, false, false );
				}
			}
			
			BOOST_FOREACH( const JointWaypoint::Ptr& curr, currSolutions )
			{
				DijkstraEdge edge;
				edge.child = curr;
				BOOST_FOREACH( const JointWaypoint::Ptr& prev, prevSolutions )
				{
					edge.parent = prev;
					edge.cost = kinematics.CalculateScore( curr->joints, prev->joints );
					prev->AddEdge( edge );
				}
			}
			prevSolutions = currSolutions;
		}
		
		BOOST_FOREACH( const JointWaypoint::Ptr& cs, currSolutions )
		{
			std::cout << "Adding goal node: " << cs->joints << std::endl;
			search.AddNode( cs, false, true );
		}
		
		std::vector<DijkstraNode::Ptr> path = search.Execute();
		JointTrajectory traj;
		double sumCost = 0;
		BOOST_FOREACH( const DijkstraNode::Ptr& node, path )
		{
			JointWaypoint::Ptr waypoint = std::dynamic_pointer_cast<JointWaypoint>( node );
			traj.push_back( *waypoint );
			sumCost += node->GetCost();
		}
		std::cout << "Path found with cost " << sumCost << std::endl;
		return traj;
	}
	
	double TrajectoryGenerator::CalculateTrajectoryCost( const JointTrajectory& traj )
	{
		double sum = 0;
		for( int i = 0; i < traj.size() - 1; i++ )
		{
			sum += kinematics.CalculateScore( traj[i].joints, traj[i+1].joints );
		}
		return sum;
	}
	
}