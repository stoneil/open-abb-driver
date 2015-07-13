#include "open_abb_driver/ABBControlInterface.h"
#include "open_abb_driver/ABBFeedbackInterface.h"
#include "open_abb_driver/PoseSE3.h"
#include "open_abb_driver/ABBKinematics.h"

//ROS specific
#include <ros/ros.h>

#include <open_abb_driver/AddWaypoint.h>
#include <open_abb_driver/ClearWaypoints.h>
#include <open_abb_driver/ExecuteWaypoints.h>
#include <open_abb_driver/Ping.h>
#include <open_abb_driver/SetCartesian.h>
#include <open_abb_driver/GetCartesian.h>
#include <open_abb_driver/GetNumWaypoints.h>
#include <open_abb_driver/SetWorkObject.h>
#include <open_abb_driver/SetZone.h>
#include <open_abb_driver/SetSoftness.h>
#include <open_abb_driver/SetTool.h>
#include <open_abb_driver/SetJoints.h>
#include <open_abb_driver/GetJoints.h>
#include <open_abb_driver/SetSpeed.h>

//ROS specific, these are redundant with abb_node
//standard libary messages instead of custom messages
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#define ID_CODE_MAX 999

#define SERVER_BAD_MSG 0
#define SERVER_OK 1
#define SERVER_COLLISION 2

#define MAX_TRANS_STEP 2.0
#define MAX_ROT_STEP (0.5 * DEG2RAD)
#define MAX_J_STEP 0.5

#define NB_FREQ 200.0
#define STOP_CHECK_FREQ 25.0
#define DIST_CHECK_FREQ 100.0

#define SAFETY_FACTOR 0.90
#define MINIMUM_TRACK_DIST_TRANS 1.0 //mm
#define MAXIMUM_TRACK_DIST_TRANS 20.0 //mm
#define MINIMUM_TRACK_DIST_ORI 0.333  //deg
#define MAXIMUM_TRACK_DIST_ORI 6.66 //deg
#define INFINITY_TRACK_DIST_TRANS 1000.0 ///mm
#define INFINITY_TRACK_DIST_ORI 333.0 //deg

#define MINIMUM_NB_SPEED_TCP 1.0 //mm/s
#define MINIMUM_NB_SPEED_ORI 0.333 //deg/s

#define NUM_JOINTS 6
#define NUM_FORCES 6

#define BLOCKING 1
#define NON_BLOCKING 0

#include <boost/thread/thread.hpp>

#include <Eigen/Geometry>

namespace open_abb_driver
{
	/*! \brief Class to process the Feedback variant types */
	class FeedbackVisitor
		: public boost::static_visitor<>
	{
	public:
		
		FeedbackVisitor( tf::TransformBroadcaster& broadcaster, ros::Publisher& cp );
		
		void operator()( const JointFeedback& fb );
		void operator()( const CartesianFeedback& fb );
		
	private:
		
		tf::TransformBroadcaster& tfBroadcaster;
		ros::Publisher& cartesianPub;
		
	};
	
	class ABBDriver
	{
	public:
		ABBDriver( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
		~ABBDriver();
		
		// Service Callbacks
		bool AddWaypointCallback( AddWaypoint::Request& req, AddWaypoint::Response& res );
		bool ClearWaypointsCallback( ClearWaypoints::Request& req, ClearWaypoints::Response& res );
		bool ExecuteWaypointsCallback( ExecuteWaypoints::Request& req, ExecuteWaypoints::Response& res );
		bool GetNumWaypointsCallback( GetNumWaypoints::Request& req, GetNumWaypoints::Response& res );
		bool PingCallback( Ping::Request& req, Ping::Response& res );
		bool SetCartesianCallback( SetCartesian::Request& req, SetCartesian::Response& res );
		bool GetCartesianCallback( GetCartesian::Request& req, GetCartesian::Response& res );
		bool SetJointsCallback( SetJoints::Request& req, SetJoints::Response& res );
		bool GetJointsCallback( GetJoints::Request& req, GetJoints::Response& res );
		bool SetToolCallback( SetTool::Request& req, SetTool::Response& res );
		bool SetWorkObjectCallback( SetWorkObject::Request& req, SetWorkObject::Response& res );
		bool SetSpeedCallback( SetSpeed::Request& req, SetSpeed::Response& res );
		bool SetZoneCallback( SetZone::Request& req, SetZone::Response& res );
		bool SetSoftnessCallback( SetSoftness::Request& req, SetSoftness::Response& res );
		
		bool AddWaypoint( const JointAngles& angles, double duration );
		bool ClearWaypoints();
		bool ExecuteWaypoints();
		bool GetNumWaypoints( int& num );
		bool Ping();
		bool SetCartesian( const PoseSE3& pose );
		bool GetCartesian( PoseSE3& pose );
		bool SetJoints( const JointAngles& angles );
		bool GetJoints( JointAngles& angles );
		bool SetTool( const PoseSE3& pose );
		bool SetWorkObject( const PoseSE3& pose );
		bool SetSpeed( double linear, double orientation );
		bool SetZone( unsigned int zone );
		bool SetSoftness( const std::array<double,6>& softness );
		
	private:
		
		typedef boost::shared_mutex Mutex;
		typedef boost::unique_lock< Mutex > WriteLock;
		typedef boost::shared_lock< Mutex > ReadLock;
		
		Mutex mutex;
		
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privHandle;
		
		ros::Publisher cartesianPub;
		
		ABBControlInterface::Ptr controlInterface;
		ABBFeedbackInterface::Ptr feedbackInterface;
		ABBKinematics ikSolver;
		
		// TODO Change to Limits struct to avoid first/second confusion
		std::array< std::pair<double,double>, 6 > jointLimits;

		// Initialize the robot
		bool Initialize();
		
		// Sets up the default robot configuration
		bool ConfigureRobot();
		
		void FeedbackSpin();
		
		tf::TransformBroadcaster tfBroadcaster;
		FeedbackVisitor feedbackVisitor;
		
		ros::ServiceServer handle_AddWaypoint;
		ros::ServiceServer handle_ClearWaypoints;
		ros::ServiceServer handle_ExecuteWaypoints;
		ros::ServiceServer handle_GetNumWaypoints;
		ros::ServiceServer handle_Ping;
		ros::ServiceServer handle_SetCartesian;
		ros::ServiceServer handle_GetCartesian;
		ros::ServiceServer handle_SetJoints;
		ros::ServiceServer handle_GetJoints;
		ros::ServiceServer handle_SetTool;
		ros::ServiceServer handle_SetWorkObject;
		ros::ServiceServer handle_SetSpeed;
		ros::ServiceServer handle_SetZone;
		ros::ServiceServer handle_SetSoftness;
		
		// Robot State
		PoseSE3 currToolTrans;
		PoseSE3 currWorkTrans;
		
		boost::thread feedbackWorker;
		
	};
	
}
