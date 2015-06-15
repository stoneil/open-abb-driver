#include "open_abb_driver/ABBNode.h"

namespace open_abb_driver
{
	
	RobotController::RobotController( const ros::NodeHandle& nh, const ros::NodeHandle& ph ) 
	: nodeHandle( nh ), privHandle( ph ), feedbackVisitor( tfBroadcaster, cartesianPub )
	{
		Initialize();
		
		handle_AddWaypoint = privHandle.advertiseService("add_waypoint", &RobotController::AddWaypointCallback, this);
		handle_ClearWaypoints = privHandle.advertiseService("clear_waypoints", &RobotController::ClearWaypointsCallback, this);
		handle_ExecuteWaypoints = privHandle.advertiseService("execute_waypoints", &RobotController::ExecuteWaypointsCallback, this);
		handle_GetNumWaypoints = privHandle.advertiseService("get_num_waypoints", &RobotController::GetNumWaypointsCallback, this);
		handle_Ping = privHandle.advertiseService("ping", &RobotController::PingCallback, this);
		handle_SetCartesian = privHandle.advertiseService("set_cartesian", &RobotController::SetCartesianCallback, this);
		handle_GetCartesian = privHandle.advertiseService("get_cartesian", &RobotController::GetCartesianCallback, this);
		handle_SetJoints = privHandle.advertiseService("set_joints", &RobotController::SetJointsCallback, this);
		handle_GetJoints = privHandle.advertiseService("get_joints", &RobotController::GetJointsCallback, this);
		handle_SetTool = privHandle.advertiseService("set_tool", &RobotController::SetToolCallback, this);
		handle_SetWorkObject = privHandle.advertiseService("set_work_object", &RobotController::SetWorkObjectCallback, this);
		handle_SetSpeed = privHandle.advertiseService("set_speed", &RobotController::SetSpeedCallback, this);
		handle_SetZone = privHandle.advertiseService("set_zone", &RobotController::SetZoneCallback, this);
		handle_SetSoftness = privHandle.advertiseService("set_softness", &RobotController::SetSoftnessCallback, this );
		
		cartesianPub = privHandle.advertise<geometry_msgs::PoseStamped>( "pose", 10, false );
		feedbackWorker = boost::thread( boost::bind( &RobotController::FeedbackSpin, this ) );
	}
	
	RobotController::~RobotController() 
	{
		feedbackWorker.join();
	}
	
	bool RobotController::Initialize()
	{
		std::string robotIp;
		int robotMotionPort;
		int robotLoggerPort;
		
		ROS_INFO( "Connecting to the ABB motion server..." );
		privHandle.param<std::string>( "ip", robotIp, "192.168.125.1" );
		privHandle.param( "motion_port", robotMotionPort, 5000 );
		controlInterface = std::make_shared<ABBControlInterface>( robotIp, robotMotionPort );
		
		ROS_INFO( "Connecting to the ABB logger server..." );
		privHandle.param( "logger_port", robotLoggerPort, 5001 );
		feedbackInterface = std::make_shared<ABBFeedbackInterface>( robotIp, robotLoggerPort );
		
		ROS_INFO("Setting robot default configuration...");
		if( !ConfigureRobot() )
		{
			ROS_WARN("Not able to set the robot to default configuration.");
			return false;
		}
		
		return true;
	}
	
	void RobotController::FeedbackSpin()
	{
		while( ros::ok() )
		{
			feedbackInterface->Spin();
			bool published = false;
			while( feedbackInterface->HasFeedback() )
			{
				Feedback fb = feedbackInterface->GetFeedback();
				boost::apply_visitor( feedbackVisitor, fb );
				published = true;
			}
			
			if( published )
			{
				ros::Time now = ros::Time::now();
				
				ReadLock lock( mutex );
				PoseSE3::Vector toolVec = currToolTrans.ToVector();
				tf::Vector3 toolTranslation( toolVec[0], toolVec[1], toolVec[2] );
				tf::Quaternion toolRot( toolVec[4], toolVec[5], toolVec[6], toolVec[3] );
				tf::Transform toolTrans( toolRot, toolTranslation );
				tf::StampedTransform toolMsg( toolTrans, now, "abb_end_effector", "abb_tool" );
				tfBroadcaster.sendTransform( toolMsg );
				
				PoseSE3::Vector workVec = currWorkTrans.ToVector();
				tf::Vector3 workTranslation( workVec[0], workVec[1], workVec[2] );
				tf::Quaternion workRot( workVec[4], workVec[5], workVec[6], workVec[3] );
				tf::Transform workTrans( workRot, workTranslation );
				tf::StampedTransform workMsg( workTrans, now, "abb_base", "abb_work_object" );
				tfBroadcaster.sendTransform( workMsg );
			}
		}
	}
	
	bool RobotController::ConfigureRobot()
	{
		double defWOx,defWOy,defWOz,defWOqw,defWOqx,defWOqy,defWOqz;
		double defTx,defTy,defTz,defTqw,defTqx,defTqy,defTqz;
		std::vector<double> softness, defSoftness(6, 0.0);
		int zone;
		double speedTCP, speedORI;
		
		//WorkObject
		privHandle.param("workobject_x",defWOx,0.0);
		privHandle.param("workobject_y",defWOy,0.0);
		privHandle.param("workobject_z",defWOz,0.0);
		privHandle.param("workobject_qw",defWOqw,1.0);
		privHandle.param("workobject_qx",defWOqx,0.0);
		privHandle.param("workobject_qy",defWOqy,0.0);
		privHandle.param("workobject_qz",defWOqz,0.0);
		PoseSE3 workObj( defWOx, defWOy, defWOz, defWOqw, defWOqx, defWOqy, defWOqz );
		
		if( !SetWorkObject( workObj ) )
		{
			ROS_WARN( "Unable to set the work object." );
			return false;
		}
		
		//Tool
		privHandle.param("tool_x",defTx,0.0);
		privHandle.param("tool_y",defTy,0.0);
		privHandle.param("tool_z",defTz,0.0);
		privHandle.param("tool_qw",defTqw,1.0);
		privHandle.param("tool_qx",defTqx,0.0);
		privHandle.param("tool_qy",defTqy,0.0);
		privHandle.param("tool_qz",defTqz,0.0);
		PoseSE3 tool( defTx, defTy, defTz, defTqw, defTqx, defTqy, defTqz );
		
		if( !SetTool( tool ) )
		{
			ROS_WARN( "Unable to set the tool." );
			return false;
		}
		
		//Zone
		privHandle.param("zone",zone,1);
		if( !SetZone(zone) )
		{
			ROS_WARN( "Unable to set the tracking zone." );
			return false;
		}
		
		// Softness
		privHandle.param("softness", softness, defSoftness );
		std::array<double,6> softn;
		std::copy( softness.begin(), softness.end(), softn.begin() );
		if( !SetSoftness( softn ) )
		{
			ROS_WARN( "Unable to set the joint softness." );
		}
		
		//Speed
		privHandle.param("speed_tcp",speedTCP,0.250);
		privHandle.param("speed_ori",speedORI,0.250);
		if( !SetSpeed(speedTCP, speedORI) )
		{
			ROS_WARN( "Unable to set the speed." );
			return false;
		}
		
		// IK Weights
		std::vector<double> weights, defWeights = {1, 1, 1, 1, 1, 1};
		privHandle.param( "ik_weights", weights, defWeights );
		if( weights.size() != 6 )
		{
			ROS_ERROR( "Must specify 6 IK weights." );
		}
		ABBKinematics::JointWeights w;
		std::copy( weights.begin(), weights.end(), w.begin() );
		ikSolver.SetJointWeights( w );
		
		// IK Joint Limits
		std::vector<double> j1Lim, j2Lim, j3Lim, j4Lim, j5Lim, j6Lim;
		std::vector<double> j1Def = { -3.146, 3.146 };
		std::vector<double> j2Def = { -1.7453, 1.9199 };
		std::vector<double> j3Def = { -1.0472, 1.1345 }; // This is limit of J3 + J2 (parallelogram)
		std::vector<double> j4Def = { -3.49, 3.49 };
		std::vector<double> j5Def = { -2.0944, 2.0944 };
		std::vector<double> j6Def = { -6.9813, 6.9813 };
		privHandle.param( "joint1_limits", j1Lim, j1Def );
		privHandle.param( "joint2_limits", j2Lim, j2Def );
		privHandle.param( "joint3_limits", j3Lim, j3Def );
		privHandle.param( "joint4_limits", j4Lim, j4Def );
		privHandle.param( "joint5_limits", j5Lim, j5Def );
		privHandle.param( "joint6_limits", j6Lim, j6Def );
		
		ikSolver.SetJointLimits( 0, std::pair<double,double>( j1Lim[0], j1Lim[1] ) );
		ikSolver.SetJointLimits( 1, std::pair<double,double>( j2Lim[0], j2Lim[1] ) );
		ikSolver.SetJointLimits( 2, std::pair<double,double>( j3Lim[0], j3Lim[1] ) );
		ikSolver.SetJointLimits( 3, std::pair<double,double>( j4Lim[0], j4Lim[1] ) );
		ikSolver.SetJointLimits( 4, std::pair<double,double>( j5Lim[0], j5Lim[1] ) );
		ikSolver.SetJointLimits( 5, std::pair<double,double>( j6Lim[0], j6Lim[1] ) );
		
		return true;
	}
	
	bool RobotController::AddWaypointCallback( AddWaypoint::Request& req, AddWaypoint::Response& res )
	{
		std::array<double,6> position;
		std::copy( req.position.begin(), req.position.end(), position.begin() );
		return AddWaypoint( position, req.duration );
	}
	
	bool RobotController::AddWaypoint( const JointAngles& angles, double duration )
	{
		JointAngles a( angles );
		a[2] += a[1];
		return controlInterface->AddWaypoint( a, duration );
	}
	
	bool RobotController::ClearWaypointsCallback( ClearWaypoints::Request& req, ClearWaypoints::Response& res )
	{
		return ClearWaypoints();
	}
	
	bool RobotController::ClearWaypoints()
	{
		return controlInterface->ClearWaypoints();
	}
	
	bool RobotController::ExecuteWaypointsCallback( ExecuteWaypoints::Request& req, ExecuteWaypoints::Response& res )
	{
		return ExecuteWaypoints();
	}
	
	bool RobotController::ExecuteWaypoints()
	{
		return controlInterface->ExecuteWaypoints();
	}
	
	bool RobotController::GetNumWaypointsCallback( GetNumWaypoints::Request& req, GetNumWaypoints::Response& res )
	{
		return GetNumWaypoints( res.numWaypoints );
	}
	
	bool RobotController::GetNumWaypoints( int& num )
	{
		return controlInterface->GetNumWaypoints( num );
	}
	
	bool RobotController::PingCallback( Ping::Request& req, Ping::Response& res )
	{
		return Ping();
	}
	
	bool RobotController::Ping()
	{
		return controlInterface->Ping();
	}
	
	bool RobotController::SetCartesianCallback( SetCartesian::Request& req, SetCartesian::Response& res )
	{
		PoseSE3 tform( req.x, req.y, req.z, req.qw, req.qx, req.qy, req.qz );
		
		return SetCartesian( tform );
	}
	
	bool RobotController::SetCartesian( const PoseSE3& pose )
	{
		PoseSE3 eff = currWorkTrans*pose*currToolTrans.Inverse();
		
		std::vector<JointAngles> ikSols;
		if( !ikSolver.ComputeIK( eff, ikSols ) || ikSols.size() == 0 )
		{
			ROS_ERROR_STREAM( "Could not find inverse kinematic solution for " << eff );
			return false;
		}
		
		JointAngles current;
		GetJoints( current );
		JointAngles best = ikSolver.GetBestSolution( current, ikSols );
	
// 		std::cout << "IK: " << best[0] << " " << best[1] << " " << best[2] << " " << best[3]
// 			<< " " << best[4] << " " << best[5] << std::endl;
		
		return( SetJoints( best ) );
	}
	
	bool RobotController::GetCartesianCallback( GetCartesian::Request& req, GetCartesian::Response& res )
	{
		PoseSE3 pose;
		if( !GetCartesian( pose ) ) { return false; }
		
		PoseSE3::Vector vec = pose.ToVector();
		res.x = vec[0];
		res.y = vec[1];
		res.z = vec[2];
		res.qw = vec[3];
		res.qx = vec[4];
		res.qy = vec[5];
		res.qz = vec[6];
		return true;
	}
	
	bool RobotController::GetCartesian( PoseSE3& pose )
	{
		JointAngles angles;
		
		if( !GetJoints( angles ) ) { return false; }
		PoseSE3 fwd = ABBKinematics::ComputeFK( angles );
 		pose = currWorkTrans.Inverse()*fwd*currToolTrans;
		return true;
	}
	
	bool RobotController::SetJointsCallback( SetJoints::Request& req, SetJoints::Response& res )
	{	
		// ROS currently uses boost::array, so we have to copy it to maintain compatibility
		std::array<double,6> position;
		std::copy( req.position.begin(), req.position.end(), position.begin() );
		return SetJoints( position );
	}
	
	bool RobotController::SetJoints( const JointAngles& angles )
	{
		JointAngles a( angles );
		a[2] += a[1];
		return controlInterface->SetJoints( a );
	}
	
	bool RobotController::GetJointsCallback( GetJoints::Request& req, GetJoints::Response& res )
	{
		std::array<double,6> position;
		if( GetJoints(position) )
		{
			std::copy( position.begin(), position.end(), res.joints.begin() );
			return true;
		}
		else { return false; }
	}
	
	bool RobotController::GetJoints( JointAngles& angles )
	{
		if( !controlInterface->GetJoints( angles ) ) { return false; }
		angles[2] -= angles[1];
		return true;
	}
	
	bool RobotController::SetToolCallback( SetTool::Request& req, SetTool::Response& res )
	{
		PoseSE3 tool( req.x, req.y, req.z, req.qw, req.qx, req.qy, req.qz );
		return SetTool( tool );
	}
	
	bool RobotController::SetTool( const PoseSE3& pose )
	{
		WriteLock lock( mutex );
		currToolTrans = pose;
		PoseSE3::Vector vec = currToolTrans.ToVector();
		return controlInterface->SetTool( vec[0], vec[1], vec[2], vec[3], vec[4], vec[5], vec[6] );
	}
	
	bool RobotController::SetWorkObjectCallback( SetWorkObject::Request& req, SetWorkObject::Response& res )
	{
		PoseSE3 work( req.x, req.y, req.z, req.qw, req.qx, req.qy, req.qz );
		return SetWorkObject( work );
	}
	
	bool RobotController::SetWorkObject( const PoseSE3& pose )
	{
		WriteLock lock( mutex );
		currWorkTrans = pose;
		PoseSE3::Vector vec = currWorkTrans.ToVector();
		return controlInterface->SetWorkObject( vec[0], vec[1], vec[2], vec[3], vec[4], vec[5], vec[6] );
	}
	
	bool RobotController::SetSpeedCallback( SetSpeed::Request& req, SetSpeed::Response& res )
	{
		return SetSpeed(req.tcp, req.ori);
	}
	
	bool RobotController::SetSpeed( double linear, double orientation )
	{
		return controlInterface->SetSpeed( linear, orientation );
	}
	
	bool RobotController::SetZoneCallback( SetZone::Request& req, SetZone::Response& res )
	{
		return SetZone(req.mode);
	}
	
	bool RobotController::SetZone( unsigned int zone )
	{
		return controlInterface->SetZone( zone );
	}
	
	bool RobotController::SetSoftnessCallback( SetSoftness::Request& req, SetSoftness::Response& res )
	{
		std::array<double,6> softness;
		std::copy( req.softness.begin(), req.softness.end(), softness.begin() );
		return SetSoftness( softness );
	}
	
	bool RobotController::SetSoftness( const std::array<double,6>& softness )
	{
		return controlInterface->SetSoftness( softness );
	}
	
	FeedbackVisitor::FeedbackVisitor( tf::TransformBroadcaster& broadcaster, ros::Publisher& cb )
		: tfBroadcaster( broadcaster ), cartesianPub( cb )
	{}
	
	void FeedbackVisitor::operator()( const JointFeedback& fb )
	{
		ros::Time now = ros::Time::now();
		JointAngles angles = fb.joints;
		angles[2] -= angles[1];
		
		PoseSE3 fwd = ABBKinematics::ComputeFK( angles );
		PoseSE3::Vector fwdv = fwd.ToVector(); //[x,y,z,qw,qx,qy,qz]
		
		tf::Vector3 translation( fwdv[0], fwdv[1], fwdv[2] );
		tf::Quaternion quat( fwdv[4], fwdv[5], fwdv[6], fwdv[3] );
		tf::Transform transform( quat, translation );
		tf::StampedTransform msg( transform, now, "abb_base", "abb_end_effector" );
		tfBroadcaster.sendTransform( msg );
		
		geometry_msgs::PoseStamped poseMsg;
		poseMsg.header.stamp = now;
		poseMsg.pose.position.x = fwdv[0];
		poseMsg.pose.position.y = fwdv[1];
		poseMsg.pose.position.z = fwdv[2];
		poseMsg.pose.orientation.w = fwdv[3];
		poseMsg.pose.orientation.x = fwdv[4];
		poseMsg.pose.orientation.y = fwdv[5];
		poseMsg.pose.orientation.z = fwdv[6];
		cartesianPub.publish( poseMsg );
	}
	
	// DEPRECATED
	void FeedbackVisitor::operator()( const CartesianFeedback& fb )
	{
// 		geometry_msgs::PoseStamped poseMsg;
// 		poseMsg.header.stamp = ros::Time::now();
// 		poseMsg.pose.position.x = fb.x;
// 		poseMsg.pose.position.y = fb.y;
// 		poseMsg.pose.position.z = fb.z;
// 		poseMsg.pose.orientation.w = fb.qw;
// 		poseMsg.pose.orientation.x = fb.qx;
// 		poseMsg.pose.orientation.y = fb.qy;
// 		poseMsg.pose.orientation.z = fb.qz;
// 		cartesianPub.publish( poseMsg );
	}
	
}

using namespace open_abb_driver;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	RobotController ABBrobot( nh, ph );
	
	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();
	
	return 0;
}
