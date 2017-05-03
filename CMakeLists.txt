cmake_minimum_required(VERSION 2.8.3)
project(open_abb_driver)

set( CMAKE_CXX_FLAGS "-g -Wall -std=c++11" )
set( CMAKE_BUILD_TYPE Release )

find_package( catkin 	REQUIRED
	COMPONENTS 			roscpp
						message_generation
						geometry_msgs
						tf
						argus_utils )

add_service_files(
	FILES 			AddWaypoint.srv
					ClearWaypoints.srv
					ExecuteWaypoints.srv
					GetCartesian.srv
					GetNumWaypoints.srv
					GetJoints.srv
					Ping.srv
					SetCartesian.srv
					SetCartesianLinear.srv
					SetDIO.srv
					SetJoints.srv
					SetSpeed.srv
					SetTool.srv
					SetWorkObject.srv
					SetZone.srv
					SetSoftness.srv
)

generate_messages( DEPENDENCIES geometry_msgs )

catkin_package(
	INCLUDE_DIRS	include
	
	LIBRARIES 		abb_comm
					
	CATKIN_DEPENDS 	roscpp
					geometry_msgs
					tf
					argus_utils
					
	DEPENDS 		Eigen3
)

include_directories( include
					 ${catkin_INCLUDE_DIRS} )

add_library( abb_comm 
			 src/ABBProtocol.cpp
			 src/ABBControlInterface.cpp
			 src/ABBFeedbackInterface.cpp
			 src/IKFastSolver.cpp
			 src/ABBKinematics.cpp
			 src/TrajectoryGenerator.cpp )
			 
add_dependencies( abb_comm ${open_abb_driver_EXPORTED_TARGETS} )
target_link_libraries( abb_comm	${catkin_LIBRARIES} )

add_executable( abb_node nodes/ABBNode.cpp )
target_link_libraries( abb_node abb_comm ${catkin_LIBRARIES} )

# add_executable( execute_csv apps/ExecuteCSV.cpp )
# target_link_libraries( execute_csv abb_comm ${catkin_LIBRARIES} )

install( TARGETS abb_comm abb_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
