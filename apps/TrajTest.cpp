#include "open_abb_driver/TrajectoryGenerator.h"

using namespace open_abb_driver;

int main( int argc, char** argv )
{
	CartesianWaypoint wp;
	wp.pose = PoseSE3( 0.8, 0.0, 1.0, 0.0, 0.0, 0.0 );
	
	CartesianWaypoint np;
	np.pose = PoseSE3( 0.8, 0.0, 1.1, 0.0, 0.0, 0.0 );
	
	CartesianTrajectory ctraj;
	ctraj.push_back( wp );
	ctraj.push_back( np );
	
	TrajectoryGenerator gen;
	std::cout << "Generating trajectory..." << std::endl;
	JointTrajectory jtraj = gen.GenerateTrajectory( ctraj );
	std::cout << "Generated trajectory of size " << jtraj.size() << std::endl;
	for( unsigned int i = 0; i < jtraj.size(); i++ )
	{
		std::cout << "WP " << i << ": " << jtraj[i].joints << std::endl;
	}
	
	return 0;
}