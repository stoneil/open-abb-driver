#include "open_abb_driver/ABBControlInterface.h"
#include "open_abb_driver/ABBFeedbackInterface.h"
#include "open_abb_driver/ABBKinematics.h"
#include "open_abb_driver/TrajectoryGenerator.h"

#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/synchronization/SynchronizationTypes.h"

//ROS specific
#include <ros/ros.h>

#include <open_abb_driver/SetCartesianTrajectory.h>
#include <open_abb_driver/AddWaypoint.h>
#include <open_abb_driver/ClearWaypoints.h>
#include <open_abb_driver/ExecuteWaypoints.h>
#include <open_abb_driver/Ping.h>
#include <open_abb_driver/SetCartesian.h>
#include <open_abb_driver/SetCartesianLinear.h>
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
	class ABBDriver
	{
	public:
		ABBDriver( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
		~ABBDriver();
		
		// Direct function calls are synchronized
		bool AddWaypoint( const JointAngles& angles, double duration );
		bool ClearWaypoints();
		// NOTE Make sure there are waypoints or else the arm errors!
		bool ExecuteWaypoints();
		bool GetNumWaypoints( int& num );
		bool Ping();
		bool SetCartesian( const argus::PoseSE3& pose );
		bool SetCartesianLinear( const argus::PoseSE3& pose, double duration, 
		                         unsigned int numWaypoints );
		bool SetCartesianTrajectory( const CartesianTrajectory& traj );
		bool GetCartesian( argus::PoseSE3& pose );

		// NOTE: Joint methods compensate for IKFast joint 1 and 2 linkage
		bool SetJoints( const JointAngles& angles );
		bool GetJoints( JointAngles& angles );

		bool SetTool( const argus::PoseSE3& pose );
		bool SetWorkObject( const argus::PoseSE3& pose );
		bool SetSpeed( double linear, double orientation );
		bool SetZone( unsigned int zone );
		bool SetSoftness( const std::array<double,6>& softness );
		
	private:
		
		typedef argus::RecursiveMutex Mutex;
		typedef argus::RecursiveLock Lock;

		mutable Mutex _armMutex; // Mutex protecting access to arm for atomic operations

		ros::NodeHandle _nodeHandle;
		ros::NodeHandle _privHandle;
		
		ros::Publisher _cartesianPub;
		
		ABBControlInterface::Ptr _controlInterface;
		ABBFeedbackInterface::Ptr _feedbackInterface;
		ABBKinematics::Ptr _ikSolver;
		TrajectoryGenerator::Ptr _trajPlanner;
		
		// TODO Change to Limits struct to avoid first/second confusion
		std::array< std::pair<double,double>, 6 > _jointLimits;

		// TODO Move to helper file
		static bool InterpolateLinearTrajectory( CartesianTrajectory& traj,
		                                         const std::vector<unsigned int>& numPoints );

		// Service Callbacks
		bool SetCartesianTrajectoryCallback( SetCartesianTrajectory::Request& req,
		                                     SetCartesianTrajectory::Response& res);
		bool AddWaypointCallback( AddWaypoint::Request& req, AddWaypoint::Response& res );
		bool ClearWaypointsCallback( ClearWaypoints::Request& req, ClearWaypoints::Response& res );
		bool ExecuteWaypointsCallback( ExecuteWaypoints::Request& req, ExecuteWaypoints::Response& res );
		bool GetNumWaypointsCallback( GetNumWaypoints::Request& req, GetNumWaypoints::Response& res );
		bool PingCallback( Ping::Request& req, Ping::Response& res );
		bool SetCartesianCallback( SetCartesian::Request& req, SetCartesian::Response& res );
		bool SetCartesianLinearCallback( SetCartesianLinear::Request& req, 
		                                 SetCartesianLinear::Response& res );
		bool GetCartesianCallback( GetCartesian::Request& req, GetCartesian::Response& res );
		bool SetJointsCallback( SetJoints::Request& req, SetJoints::Response& res );
		bool GetJointsCallback( GetJoints::Request& req, GetJoints::Response& res );
		bool SetToolCallback( SetTool::Request& req, SetTool::Response& res );
		bool SetWorkObjectCallback( SetWorkObject::Request& req, SetWorkObject::Response& res );
		bool SetSpeedCallback( SetSpeed::Request& req, SetSpeed::Response& res );
		bool SetZoneCallback( SetZone::Request& req, SetZone::Response& res );
		bool SetSoftnessCallback( SetSoftness::Request& req, SetSoftness::Response& res );
		
		// Initialize the robot
		bool Initialize();
		
		// Sets up the default robot configuration
		bool ConfigureRobot();
		
		void FeedbackSpin();
		
		tf::TransformBroadcaster _tfBroadcaster;
		
		// Service servers for all ROS services
		ros::ServiceServer _setCartesianTrajectoryServer;
		ros::ServiceServer _addWaypointServer;
		ros::ServiceServer _clearWaypointsServer;
		ros::ServiceServer _executeWaypointsServer;
		ros::ServiceServer _getNumWaypointsServer;
		ros::ServiceServer _pingServer;
		ros::ServiceServer _setCartesianServer;
		ros::ServiceServer _setCartesianLinearServer;
		ros::ServiceServer _getCartesianServer;
		ros::ServiceServer _setJointsServer;
		ros::ServiceServer _getJointsServer;
		ros::ServiceServer _setToolServer;
		ros::ServiceServer _setWorkObjectServer;
		ros::ServiceServer _setSpeedServer;
		ros::ServiceServer _setZoneServer;
		ros::ServiceServer _setSoftnessServer;
		
		boost::thread _feedbackWorker; // Spinner for feedback interface

		// Local cached robot properties and synchronized getters/setters
		argus::PoseSE3 GetToolTrans() const;
		void SetToolTrans( const argus::PoseSE3& pose );
		argus::PoseSE3 GetWorkTrans() const;
		void SetWorkTrans( const argus::PoseSE3& pose );

		mutable Mutex _cacheMutex; // Mutex protecting access to local cached properties
		argus::PoseSE3 _currToolTrans;
		argus::PoseSE3 _currWorkTrans;	
	};
	
}
