#ifndef RTT_MOTION_PRIMITIVE_WRAPPER_HPP
#define RTT_MOTION_PRIMITIVE_WRAPPER_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/base/RunnableInterface.hpp>
#include <rtt/Activity.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

#include <rci/dto/JointAngles.h>
#include <rci/dto/JointTorques.h>
#include <rci/dto/JointVelocities.h>
#include <rci/dto/JointAccelerations.h>
#include <rci/dto/CartesianPose.h>


#include "CartesianSpace_CircularTask.hpp"


class RttMotionPrimitiveWrapper: public RTT::TaskContext {
public:
	RttMotionPrimitiveWrapper(std::string const& name);
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
	/**
	 * InputPorts read data.
	 */
	RTT::InputPort<rci::JointAnglesPtr> currJntPos_Port;
	RTT::FlowStatus currJntPos_Flow;
};
#endif
