#include "open_abb_driver/ABBNode.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"

using namespace argus;

namespace open_abb_driver
{
	
ABBDriver::ABBDriver( const ros::NodeHandle& nh, const ros::NodeHandle& ph ) 
: _nodeHandle( nh ), _privHandle( ph )
{
	Initialize();
	
	_addWaypointServer = _privHandle.advertiseService( "add_waypoint", 
	                                                   &ABBDriver::AddWaypointCallback, 
	                                                   this);
	_clearWaypointsServer = _privHandle.advertiseService( "clear_waypoints", 
	                                                      &ABBDriver::ClearWaypointsCallback, 
	                                                      this);
	_executeWaypointsServer = _privHandle.advertiseService( "execute_waypoints", 
	                                                        &ABBDriver::ExecuteWaypointsCallback, 
	                                                        this);
	_getNumWaypointsServer = _privHandle.advertiseService( "get_num_waypoints", 
	                                                       &ABBDriver::GetNumWaypointsCallback, 
	                                                       this);
	_pingServer = _privHandle.advertiseService( "ping", 
	                                            &ABBDriver::PingCallback, 
	                                            this);
	_setCartesianServer = _privHandle.advertiseService( "set_cartesian", 
	                                                    &ABBDriver::SetCartesianCallback, 
	                                                    this);
	_setCartesianLinearServer = _privHandle.advertiseService( "set_cartesian_linear", 
	                                                          &ABBDriver::SetCartesianLinearCallback, 
	                                                          this);
	_getCartesianServer = _privHandle.advertiseService( "get_cartesian", 
	                                                    &ABBDriver::GetCartesianCallback, 
	                                                    this);
	_setJointsServer = _privHandle.advertiseService( "set_joints", 
	                                                 &ABBDriver::SetJointsCallback, 
	                                                 this);
	_getJointsServer = _privHandle.advertiseService( "get_joints", 
	                                                 &ABBDriver::GetJointsCallback, 
	                                                 this);
	_setToolServer = _privHandle.advertiseService( "set_tool", 
	                                               &ABBDriver::SetToolCallback, 
	                                               this );
	_setWorkObjectServer = _privHandle.advertiseService( "set_work_object", 
	                                                     &ABBDriver::SetWorkObjectCallback, 
	                                                     this);
	_setSpeedServer = _privHandle.advertiseService( "set_speed", 
	                                                &ABBDriver::SetSpeedCallback, 
	                                                this);
	_setZoneServer = _privHandle.advertiseService( "set_zone", 
	                                               &ABBDriver::SetZoneCallback, 
	                                               this);
	_setSoftnessServer = _privHandle.advertiseService( "set_softness", 
	                                                   &ABBDriver::SetSoftnessCallback, 
	                                                   this );
	
	_cartesianPub = _privHandle.advertise<geometry_msgs::PoseStamped>( "pose", 
	                                                                   10, 
	                                                                   false );
	_feedbackWorker = boost::thread( boost::bind( &ABBDriver::FeedbackSpin, this ) );
}

ABBDriver::~ABBDriver() 
{
	_feedbackWorker.join();
}

bool ABBDriver::Initialize()
{
	std::string robotIp;
	int robotMotionPort;
	int robotLoggerPort;
	
	ROS_INFO( "Connecting to the ABB motion server..." );
	GetParam<std::string>( _privHandle, "ip", robotIp, "192.168.125.1" );
	GetParam( _privHandle, "motion_port", robotMotionPort, 5000 );
	_controlInterface = std::make_shared<ABBControlInterface>( robotIp, robotMotionPort );
	
	ROS_INFO( "Connecting to the ABB logger server..." );
	GetParam( _privHandle, "logger_port", robotLoggerPort, 5001 );
	_feedbackInterface = std::make_shared<ABBFeedbackInterface>( robotIp, robotLoggerPort );
	
	ROS_INFO("Setting robot default configuration...");
	if( !ConfigureRobot() )
	{
		ROS_WARN("Not able to set the robot to default configuration.");
		return false;
	}
	
	return true;
}

void ABBDriver::FeedbackSpin()
{
	while( ros::ok() )
	{
		_feedbackInterface->Spin();
		bool published = false;

		while( _feedbackInterface->HasFeedback() )
		{
			JointFeedback fb = _feedbackInterface->GetFeedback();

			// TODO Currently no way to synchronize clocks with ABB arm
			ros::Time now = ros::Time::now();
			JointAngles angles = fb.joints;
			angles[2] -= angles[1];
			
			// TODO Make this cleaner
			PoseSE3 fwd = ABBKinematics::ComputeFK( angles );
			fwd = _currWorkTrans.Inverse()*fwd*_currToolTrans;
			
			FixedVectorType<7> fwdv = fwd.ToVector();
			tf::Vector3 translation( fwdv[0], fwdv[1], fwdv[2] );
			tf::Quaternion quat( fwdv[4], fwdv[5], fwdv[6], fwdv[3] );
			tf::Transform transform( quat, translation );
			tf::StampedTransform msg( transform, now, "abb_base", "abb_end_effector" );
			_tfBroadcaster.sendTransform( msg );
			
			geometry_msgs::PoseStamped poseMsg;
			poseMsg.pose = PoseToMsg( fwd );
			poseMsg.header.stamp = now;
			_cartesianPub.publish( poseMsg );
			published = true;
		}
		
		if( published )
		{
			ros::Time now = ros::Time::now();
			
			ReadLock lock( _mutex );
			argus::FixedVectorType<7> toolVec = _currToolTrans.ToVector();
			tf::Vector3 toolTranslation( toolVec[0], toolVec[1], toolVec[2] );
			tf::Quaternion toolRot( toolVec[4], toolVec[5], toolVec[6], toolVec[3] );
			tf::Transform toolTrans( toolRot, toolTranslation );
			tf::StampedTransform toolMsg( toolTrans, now, "abb_end_effector", "abb_tool" );
			_tfBroadcaster.sendTransform( toolMsg );
			
			argus::FixedVectorType<7> workVec = _currWorkTrans.ToVector();
			tf::Vector3 workTranslation( workVec[0], workVec[1], workVec[2] );
			tf::Quaternion workRot( workVec[4], workVec[5], workVec[6], workVec[3] );
			tf::Transform workTrans( workRot, workTranslation );
			tf::StampedTransform workMsg( workTrans, now, "abb_base", "abb_work_object" );
			_tfBroadcaster.sendTransform( workMsg );
		}
	}
}

bool ABBDriver::ConfigureRobot()
{
	//WorkObject
	PoseSE3 workObj;
	GetParamRequired( _privHandle, "work_object_pose", workObj );
	if( !SetWorkObject( workObj ) )
	{
		ROS_WARN( "Unable to set the work object." );
		return false;
	}
	
	//Tool
	PoseSE3 tool;
	GetParamRequired( _privHandle, "tool_pose", tool );
	if( !SetTool( tool ) )
	{
		ROS_WARN( "Unable to set the tool." );
		return false;
	}
	
	//Zone
	int zone;
	GetParam( _privHandle, "zone", zone, 1 );
	if( !SetZone(zone) )
	{
		ROS_WARN( "Unable to set the tracking zone." );
		return false;
	}
	
	// Softness
	std::vector<double> softness;
	if( GetParam( _privHandle, "softness", softness ) )
	{
		std::array<double,6> softn;
		std::copy( softness.begin(), softness.end(), softn.begin() );
		if( !SetSoftness( softn ) )
		{
			ROS_WARN( "Unable to set the joint softness." );
		}
	}
	
	//Speed
	double speedTCP, speedORI;
	GetParam( _privHandle, "speed_tcp", speedTCP, 0.250 );
	GetParam( _privHandle, "speed_ori", speedORI, 0.250 );
	if( !SetSpeed(speedTCP, speedORI) )
	{
		ROS_WARN( "Unable to set the speed." );
		return false;
	}
	
	// IK Weights
	std::vector<double> weights, defWeights = {1, 1, 1, 1, 1, 1};
	if( !GetParam( _privHandle, "ik_weights", weights ) )
	{
		weights = defWeights;
	}
	if( weights.size() != 6 )
	{
		ROS_ERROR( "Must specify 6 IK weights." );
	}
	ABBKinematics::JointWeights w;
	std::copy( weights.begin(), weights.end(), w.begin() );

	_ikSolver = std::make_shared<ABBKinematics>();
	_ikSolver->SetJointWeights( w );

	_trajPlanner = std::make_shared<TrajectoryGenerator>( _ikSolver );
	
	// IK Joint Limits
	std::vector<double> j1Lim = { -3.146, 3.146 };
	std::vector<double> j2Lim = { -1.45, 1.9199 };
	std::vector<double> j3Lim = { -1.0472, 1.1345 }; // This is limit of J3 + J2 (parallelogram)
	std::vector<double> j4Lim = { -3.49, 3.49 };
	std::vector<double> j5Lim = { -2.0944, 2.0944 };
	std::vector<double> j6Lim = { -6.9813, 6.9813 };
	GetParam( _privHandle, "joint1_limits", j1Lim );
	GetParam( _privHandle, "joint2_limits", j2Lim );
	GetParam( _privHandle, "joint3_limits", j3Lim );
	GetParam( _privHandle, "joint4_limits", j4Lim );
	GetParam( _privHandle, "joint5_limits", j5Lim );
	GetParam( _privHandle, "joint6_limits", j6Lim );
	
	_jointLimits[0] = std::pair<double,double>( j1Lim[0], j1Lim[1] );
	_jointLimits[1] = std::pair<double,double>( j2Lim[0], j2Lim[1] );
	_jointLimits[2] = std::pair<double,double>( j3Lim[0], j3Lim[1] );
	_jointLimits[3] = std::pair<double,double>( j4Lim[0], j4Lim[1] );
	_jointLimits[4] = std::pair<double,double>( j5Lim[0], j5Lim[1] );
	_jointLimits[5] = std::pair<double,double>( j6Lim[0], j6Lim[1] );
	
	_ikSolver->SetJointLimits( 0, _jointLimits[0] );
	_ikSolver->SetJointLimits( 1, _jointLimits[1] );
	_ikSolver->SetJointLimits( 2, _jointLimits[2] );
	_ikSolver->SetJointLimits( 3, _jointLimits[3] );
	_ikSolver->SetJointLimits( 4, _jointLimits[4] );
	_ikSolver->SetJointLimits( 5, _jointLimits[5] );
	
	return true;
}

bool ABBDriver::AddWaypointCallback( AddWaypoint::Request& req, AddWaypoint::Response& res )
{
	std::array<double,6> position;
	std::copy( req.position.begin(), req.position.end(), position.begin() );
	return AddWaypoint( position, req.duration );
}

bool ABBDriver::AddWaypoint( const JointAngles& angles, double duration )
{
	JointAngles a( angles );
	a[2] += a[1];
	return _controlInterface->AddWaypoint( a, duration );
}

bool ABBDriver::ClearWaypointsCallback( ClearWaypoints::Request& req, ClearWaypoints::Response& res )
{
	return ClearWaypoints();
}

bool ABBDriver::ClearWaypoints()
{
	return _controlInterface->ClearWaypoints();
}

bool ABBDriver::ExecuteWaypointsCallback( ExecuteWaypoints::Request& req, ExecuteWaypoints::Response& res )
{
	return ExecuteWaypoints();
}

bool ABBDriver::ExecuteWaypoints()
{
	return _controlInterface->ExecuteWaypoints();
}

bool ABBDriver::GetNumWaypointsCallback( GetNumWaypoints::Request& req, GetNumWaypoints::Response& res )
{
	return GetNumWaypoints( res.numWaypoints );
}

bool ABBDriver::GetNumWaypoints( int& num )
{
	return _controlInterface->GetNumWaypoints( num );
}

bool ABBDriver::PingCallback( Ping::Request& req, Ping::Response& res )
{
	return Ping();
}

bool ABBDriver::Ping()
{
	return _controlInterface->Ping();
}

bool ABBDriver::SetCartesianCallback( SetCartesian::Request& req, SetCartesian::Response& res )
{
	PoseSE3 tform( req.x, req.y, req.z, req.qw, req.qx, req.qy, req.qz );
	
	return SetCartesian( tform );
}

bool ABBDriver::SetCartesianLinearCallback( SetCartesianLinear::Request& req,
                                            SetCartesianLinear::Response& res )
{
	PoseSE3 tform = MsgToPose( req.pose );
	PoseSE3 targetPose = _currWorkTrans * tform * _currToolTrans.Inverse();

	PoseSE3 currentPose;
	if( !GetCartesian( currentPose ) )
	{
		ROS_ERROR_STREAM( "Could not get current end effector pose." );
		return false;
	}
	currentPose = _currWorkTrans * currentPose * _currToolTrans.Inverse();

	// PoseSE3 movement = targetPose.Inverse() * currentPose;
	PoseSE3 movement = currentPose.Inverse() * targetPose;
	PoseSE3::TangentVector bodyVel = PoseSE3::Log( movement );
	PoseSE3 delta = PoseSE3::Exp( bodyVel / ( req.num_waypoints ) );
	double dt = req.duration / req.num_waypoints;

	// ROS_INFO_STREAM( "Current pose: " << currentPose );
	// ROS_INFO_STREAM( "Target pose: " << targetPose );
	// ROS_INFO_STREAM( "Body velocity: " << bodyVel.transpose() );

	CartesianTrajectory traj;
	traj.reserve( req.num_waypoints );
	PoseSE3 acc = currentPose;
	double tAcc = 0;
	for( unsigned int i = 0; i < req.num_waypoints; i++ )
	{
		acc = acc * delta;
		tAcc = tAcc + dt;
		CartesianWaypoint wp;
		wp.pose = acc;
		wp.time = ros::Duration( tAcc );
		// ROS_INFO_STREAM( "Waypoint pose: " << wp.pose );
		traj.push_back( wp );
	}

	JointAngles currentJoints;
	if( !GetJoints( currentJoints ) )
	{
		ROS_ERROR_STREAM( "Could not get current joint values." );
		return false;
	}
	JointTrajectory jTraj;
	try
	{
		jTraj = _trajPlanner->GenerateTrajectory( currentJoints, traj );
	}
	catch( std::runtime_error& e )
	{
		ROS_ERROR_STREAM( "Could not find trajectory: " << e.what() );
		return false;
	}
	if( jTraj.size() != req.num_waypoints + 1 )
	{
		ROS_WARN_STREAM( "Planned trajectory has " << jTraj.size() - 1 << 
			             " waypoints instead of requested " << req.num_waypoints );
		return false;
	}

	if( !_controlInterface->ClearWaypoints() )
	{
		ROS_ERROR_STREAM( "Could not clear waypoint buffer." );
		return false;
	}
	// Skip the first waypoint since we are "already there"
	for( unsigned int i = 1; i < jTraj.size(); ++i )
	{
		// ROS_INFO_STREAM( "Adding waypoint: " << jTraj[i].joints << " dt: " << dt );
		if( !AddWaypoint( jTraj[i].joints, dt ) )
		{
			ROS_ERROR_STREAM( "Could not add waypoint to buffer." );
			return false;
		}
	}
	return _controlInterface->ExecuteWaypoints();
}

bool ABBDriver::SetCartesian( const PoseSE3& pose )
{
	PoseSE3 eff = _currWorkTrans*pose*_currToolTrans.Inverse();
	
	std::vector<JointAngles> ikSols;
	if( !_ikSolver->ComputeIK( eff, ikSols ) || ikSols.size() == 0 )
	{
		ROS_ERROR_STREAM( "Could not find inverse kinematic solution for " << eff );
		return false;
	}
	
	JointAngles current;
	GetJoints( current );
	JointAngles best = _ikSolver->GetBestSolution( current, ikSols );

// 		std::cout << "IK: " << best[0] << " " << best[1] << " " << best[2] << " " << best[3]
// 			<< " " << best[4] << " " << best[5] << std::endl;
	
	return( SetJoints( best ) );
}

bool ABBDriver::GetCartesianCallback( GetCartesian::Request& req, GetCartesian::Response& res )
{
	PoseSE3 pose;
	if( !GetCartesian( pose ) ) { return false; }
	
	argus::FixedVectorType<7> vec = pose.ToVector();
	res.x = vec[0];
	res.y = vec[1];
	res.z = vec[2];
	res.qw = vec[3];
	res.qx = vec[4];
	res.qy = vec[5];
	res.qz = vec[6];
	return true;
}

bool ABBDriver::GetCartesian( PoseSE3& pose )
{
	JointAngles angles;
	
	if( !GetJoints( angles ) ) { return false; }
	PoseSE3 fwd = ABBKinematics::ComputeFK( angles );
	pose = _currWorkTrans.Inverse()*fwd*_currToolTrans;
	return true;
}

bool ABBDriver::SetJointsCallback( SetJoints::Request& req, SetJoints::Response& res )
{	
	// ROS currently uses boost::array, so we have to copy it to maintain compatibility
	std::array<double,6> position;
	std::copy( req.position.begin(), req.position.end(), position.begin() );
	return SetJoints( position );
}

bool ABBDriver::SetJoints( const JointAngles& angles )
{
	JointAngles a( angles );
	a[2] += a[1];
	
	for( unsigned int i = 0; i < 6; i++ )
	{
		if( angles[i] < _jointLimits[i].first || angles[i] > _jointLimits[i].second )
		{
			ROS_ERROR_STREAM( "Commanded joint " << i+1 << " angle of " << angles[i] << " exceeds limits ("
				<< _jointLimits[i].first << ", " << _jointLimits[i].second << ")." );
			return false;
		}
	}
	return _controlInterface->SetJoints( a );
}

bool ABBDriver::GetJointsCallback( GetJoints::Request& req, GetJoints::Response& res )
{
	std::array<double,6> position;
	if( GetJoints(position) )
	{
		std::copy( position.begin(), position.end(), res.joints.begin() );
		return true;
	}
	else { return false; }
}

bool ABBDriver::GetJoints( JointAngles& angles )
{
	if( !_controlInterface->GetJoints( angles ) ) { return false; }
	angles[2] -= angles[1];
	return true;
}

bool ABBDriver::SetToolCallback( SetTool::Request& req, SetTool::Response& res )
{
	PoseSE3 tool( req.x, req.y, req.z, req.qw, req.qx, req.qy, req.qz );
	return SetTool( tool );
}

bool ABBDriver::SetTool( const PoseSE3& pose )
{
	WriteLock lock( _mutex );
	_currToolTrans = pose;
	argus::FixedVectorType<7> vec = _currToolTrans.ToVector();
	return _controlInterface->SetTool( vec[0], vec[1], vec[2], vec[3], vec[4], vec[5], vec[6] );
}

bool ABBDriver::SetWorkObjectCallback( SetWorkObject::Request& req, SetWorkObject::Response& res )
{
	PoseSE3 work( req.x, req.y, req.z, req.qw, req.qx, req.qy, req.qz );
	return SetWorkObject( work );
}

bool ABBDriver::SetWorkObject( const PoseSE3& pose )
{
	WriteLock lock( _mutex );
	_currWorkTrans = pose;
	argus::FixedVectorType<7> vec = _currWorkTrans.ToVector();
	return _controlInterface->SetWorkObject( vec[0], vec[1], vec[2], vec[3], vec[4], vec[5], vec[6] );
}

bool ABBDriver::SetSpeedCallback( SetSpeed::Request& req, SetSpeed::Response& res )
{
	return SetSpeed(req.tcp, req.ori);
}

bool ABBDriver::SetSpeed( double linear, double orientation )
{
	return _controlInterface->SetSpeed( linear, orientation );
}

bool ABBDriver::SetZoneCallback( SetZone::Request& req, SetZone::Response& res )
{
	return SetZone(req.mode);
}

bool ABBDriver::SetZone( unsigned int zone )
{
	return _controlInterface->SetZone( zone );
}

bool ABBDriver::SetSoftnessCallback( SetSoftness::Request& req, SetSoftness::Response& res )
{
	std::array<double,6> softness;
	std::copy( req.softness.begin(), req.softness.end(), softness.begin() );
	return SetSoftness( softness );
}

bool ABBDriver::SetSoftness( const std::array<double,6>& softness )
{
	return _controlInterface->SetSoftness( softness );
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "abb_driver");
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	open_abb_driver::ABBDriver ABBrobot( nh, ph );
	
	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();
	
	return 0;
}
