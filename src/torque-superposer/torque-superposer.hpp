#ifndef RTT_JPC_HPP
#define RTT_JPC_HPP
// RTT includes
#include <rtt/RTT.hpp>
#include <rtt/base/RunnableInterface.hpp>
#include <rtt/Activity.hpp>
// RCI includes
#include <rci/dto/JointAngles.h>
#include <rci/dto/JointVelocities.h>
#include <rci/dto/JointTorques.h>
#include <nemo/Vector.h>
#include <nemo/Mapping.h>
// KDL includes
#include <kdl/tree.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainfksolver.hpp>

// BOOST includes
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

class TorqueSuperposer: public RTT::TaskContext {
public:
	TorqueSuperposer(std::string const& name);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

protected:
	RTT::OutputPort<rci::JointTorquesPtr> trq_out_port;

	std::vector<boost::shared_ptr<RTT::InputPort<rci::JointTorquesPtr> > > trq_in_ports;

	std::vector<RTT::FlowStatus> trq_in_flows;

	rci::JointTorquesPtr trqCmdOutput;
	rci::JointTorquesPtr tmpTrqValue;

	int numberOfPorts;
	int fixedNumPorts;
	int dimensionOfInput;
	std::vector<double> weights;
};
#endif
