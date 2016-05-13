#ifndef RTT_MOTION_PRIMITIVE_WRAPPER_HPP
#define RTT_MOTION_PRIMITIVE_WRAPPER_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/base/RunnableInterface.hpp>
#include <rtt/Activity.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>

#include <iostream>
#include <string>

#include "CartesianSpace_CircularTask.hpp"

class RttMotionPrimitiveWrapper: public RTT::TaskContext {
public:
	RttMotionPrimitiveWrapper(std::string const& name, double start_time);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

protected:
	double getSimulationTime();
	double start_time;

	CartesianSpace_CircularTask cart_task;

	//output translation ports
	RTT::OutputPort<Eigen::Vector3d> cmdCartPosTranslation_Port;
	RTT::OutputPort<Eigen::Vector3d> cmdCartVelTranslation_Port;
	RTT::OutputPort<Eigen::Vector3d> cmdCartAccTranslation_Port;

	//output orientation ports
	RTT::OutputPort<Eigen::Vector3d> cmdCartPosOrientation_Port;
	RTT::OutputPort<Eigen::Vector3d> cmdCartVelOrientation_Port;
	RTT::OutputPort<Eigen::Vector3d> cmdCartAccOrientation_Port;

	Eigen::Vector3d cmdCartPosTranslation;
	Eigen::Vector3d cmdCartVelTranslation;
	Eigen::Vector3d cmdCartAccTranslation;
	Eigen::Vector3d cmdCartPosOrientation;
	Eigen::Vector3d cmdCartVelOrientation;
	Eigen::Vector3d cmdCartAccOrientation;

};
#endif
