#ifndef _ABB2400_IK_H_
#define _ABB2400_IK_H_

#include "ikfast/ikfast.h"
#include "argus_utils/PoseSE3.h"

#include <array>
#include <memory>

namespace open_abb_driver
{
	typedef std::array<double,6> JointAngles;
	std::ostream& operator<<( std::ostream& os, const JointAngles& );
	
	class ABBKinematics
	{
	public:
		
		typedef std::shared_ptr<ABBKinematics> Ptr;
		
		typedef std::array<double,6> JointWeights;
		
		ABBKinematics();
		
		/*! \brief Set the weights used to find the nearest solution. */
		void SetJointWeights( const JointWeights& w );
		
		/*! \brief Set the limits on the solver joints. 0-indexed. */
		void SetJointLimits( unsigned int index, std::pair<double,double> limits );
		
		bool ComputeIK( const argus_utils::PoseSE3& ref, std::vector<JointAngles>& solutions );
	
		static argus_utils::PoseSE3 ComputeFK( const JointAngles& angles );
		
		JointAngles GetBestSolution( const JointAngles& currentAngles, 
									 const std::vector<JointAngles>& solutions );
	
		double CalculateScore( const JointAngles& a, const JointAngles& b );

	private:
		
		JointWeights weights;
		std::array< std::pair<double,double>, 6 > jointLimits;
		
		
	};
	
}

#endif
