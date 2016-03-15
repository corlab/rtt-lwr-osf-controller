#ifndef RTT_CONTROLLER_COMPONENT_HPP
#define RTT_CONTROLLER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/base/RunnableInterface.hpp>
#include <rtt/Activity.hpp>

#include <rci/dto/JointAngles.h>
#include <rci/dto/JointTorques.h>
#include <rci/dto/JointVelocities.h>

#include <rsb/Factory.h>
#include <rsb/Listener.h>

boost::mutex rsbcmdJointTorques_mtx;
rci::JointTorquesPtr rsbcmdJointTorques;

class RTTController: public RTT::TaskContext {
public:
	RTTController(std::string const& name);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

protected:
	/**
	 * OutputPorts publish data.
	 */
	RTT::OutputPort<rci::JointAnglesPtr> cmdJntPos_Port;
	RTT::OutputPort<rci::JointTorquesPtr> cmdJntTrq_Port;
	RTT::OutputPort<rci::JointTorquesPtr> cmdJntTrq_Port_1;
	RTT::OutputPort<rci::JointTorquesPtr> cmdJntTrq_Port_2;
	RTT::OutputPort<rci::JointTorquesPtr> cmdJntTrq_Port_3;
	RTT::OutputPort<rci::JointTorquesPtr> cmdJntTrq_Port_4;
	RTT::OutputPort<rci::JointTorquesPtr> cmdJntTrq_Port_5;
	RTT::OutputPort<rci::JointTorquesPtr> cmdJntTrq_Port_6;
	/**
	 * InputPorts read data.
	 */
	RTT::InputPort<rci::JointAnglesPtr> currJntPos_Port;
	RTT::InputPort<rci::JointTorquesPtr> currJntTrq_Port;
	RTT::InputPort<rci::JointVelocitiesPtr> currJntVel_Port;
	/**
	 * Hold the read value
	 */
	rci::JointAnglesPtr currJntPos;
	rci::JointTorquesPtr currJntTrq;
	rci::JointVelocitiesPtr currJntVel;

	rci::JointAnglesPtr outJntPos;
	rci::JointTorquesPtr outJntTrq;

	/**
	 * Commands from the outside world
	 */
	rsb::ListenerPtr rsbcmdJointPosition_Listener;


	double gain;
};
#endif
