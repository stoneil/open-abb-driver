#ifndef ABB_INTERFACE_FEEDBACK_H_
#define ABB_INTERFACE_FEEDBACK_H_

#include <boost/thread/thread.hpp>
#include <boost/variant.hpp>
#include <boost/circular_buffer.hpp>

#include "open_abb_driver/ABBKinematics.h"

#include <memory>

namespace open_abb_driver
{

struct JointFeedback
{
	std::string date;
	std::string time;
	std::array<double,6> joints;
};

class ABBFeedbackInterface 
{
public:
	
	typedef std::shared_ptr<ABBFeedbackInterface> Ptr;
	
	
	ABBFeedbackInterface( const std::string& ip, int port, size_t bufferSize = 10 );
	~ABBFeedbackInterface();
	
	void Spin();
	bool HasFeedback() const;
	JointFeedback GetFeedback();
	
private:
	
	mutable boost::mutex mutex;
	int loggerSocket;
	
	boost::circular_buffer<JointFeedback> outgoing;
	
};

}

#endif
