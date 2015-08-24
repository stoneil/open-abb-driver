#include <ros/ros.h>
#include "open_abb_driver/TrajectoryGenerator.h"
#include "open_abb_driver/AddWaypoint.h"

#include "argus_utils/PoseSE3.h"

#include <fstream>

#include <Eigen/Dense>

using namespace open_abb_driver;
using namespace argus_utils;

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "csv_trajectory" );
	ros::NodeHandle ph( "~" );
	
	std::string csvPath;
	ph.getParam( "csv_path", csvPath );
	
	std::ifstream csv( csvPath );
	if( !csv.is_open() )
	{
		throw std::runtime_error( "Could not open csv at path: " + csvPath );
	}
	
	double ox, oy, oz, oqw, oqx, oqy, oqz;
	ph.getParam( "offset_x", ox );
	ph.getParam( "offset_y", oy );
	ph.getParam( "offset_z", oz );
	ph.getParam( "offset_qw", oqw );
	ph.getParam( "offset_qx", oqx );
	ph.getParam( "offset_qy", oqy );
	ph.getParam( "offset_qz", oqz );
	PoseSE3 offset( ox, oy, oz, oqw, oqx, oqy, oqz );
	
	double timescale;
	ph.param( "time_scale", timescale, 1.0 );
	std::cout << "ts: " << timescale << std::endl;
	
	int subsample;
	ph.param( "subsample", subsample, 1 );
	
	std::string line;
	CartesianTrajectory ctraj;
	double t, x, y, z;
	CartesianWaypoint wp;
	
	Eigen::Vector3d orig( 1.0, 0.0, 0.0 );
	Eigen::Quaterniond toolQuat;
	
	ROS_INFO( "Parsing CSV..." );
	while( !csv.eof() )
	{
		for( unsigned int i = 0; i < subsample; i++ )
		{
			std::getline( csv, line );
		}
		
		sscanf( line.c_str(), "%lf,%lf,%lf,%lf", &t, &x, &y, &z );
		t = t*timescale;
		
		PoseSE3 dummy = offset*PoseSE3( x, y, z, 1, 0, 0, 0 );
		PoseSE3::Translation dummyTrans = dummy.GetTranslation();
		Eigen::Vector3d radial( dummyTrans.x(), dummyTrans.y(), dummyTrans.z() );
		toolQuat.setFromTwoVectors( orig, radial );
		if( std::isnan( toolQuat.w() ) )
		{
			toolQuat.w() = 1.0;
			toolQuat.x() = 0.0;
			toolQuat.y() = 0.0;
			toolQuat.z() = 0.0;
		}

		PoseSE3::Translation toolTrans( x, y, z );
		wp.pose = offset*PoseSE3( toolTrans, toolQuat );
		wp.time = ros::Time( t ).toBoost();
		ctraj.push_back( wp );
		
		ROS_INFO_STREAM( "Adding cartesian waypoint " << wp.pose );
	}
	
	ROS_INFO_STREAM( "CSV parsed with " << ctraj.size() << " waypoints. Generating joint trajectory..." );
	TrajectoryGenerator gen;
	JointTrajectory jtraj = gen.GenerateTrajectory( ctraj );
	
	ros::NodeHandle nh;
	std::string armName;
	ph.getParam( "arm_name", armName );
	ros::service::waitForService( armName + "/add_waypoint" );
	ros::ServiceClient waypointClient = nh.serviceClient<AddWaypoint>( armName + "/add_waypoint", true );
	
	AddWaypoint awsrv;
	for( unsigned int i = 0; i < jtraj.size(); i++ )
	{
		std::cout << "joints[" << i << "]: " << jtraj[i].joints << std::endl;
		std::copy( jtraj[i].joints.begin(), jtraj[i].joints.end(), awsrv.request.position.begin() );
		if( i == 0 )
		{
			awsrv.request.duration = 5.0;
		}
		else
		{
			boost::posix_time::time_duration dur = jtraj[i].time - jtraj[i-1].time;
			std::cout << "Time: " << boost::posix_time::to_simple_string( jtraj[i].time ) << std::endl;
			awsrv.request.duration = dur.total_microseconds()*1E-6;
			std::cout << "Duration: " << awsrv.request.duration << std::endl;
		}
		waypointClient.call( awsrv );
	}
	
	return 0;
}
