#ifndef RTT_MOTION_PRIMITIVE_WRAPPER_HPP
#define RTT_MOTION_PRIMITIVE_WRAPPER_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/base/RunnableInterface.hpp>
#include <rtt/Activity.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>

//#include <Eigen/Dense>
//#include <Eigen/Geometry>
#include <string>

#include <rci/dto/CartesianPose.h>
//#include <rci/dto/CartesianVelocity.h>
//#include <rci/dto/CartesianAcceleration.h>

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
	double getSimulationTime();
	double start_time;

	CartesianSpace_CircularTask cart_task;

	/**
	 * OutputPorts publish data.
	 */
	RTT::OutputPort<rci::PosePtr> cmdCartPos_Port;
	RTT::OutputPort<rci::PosePtr> cmdCartVel_Port;
	RTT::OutputPort<rci::PosePtr> cmdCartAcc_Port;

	rci::PosePtr cmdCartPos;
	rci::PosePtr cmdCartVel;
	rci::PosePtr cmdCartAcc;

	Eigen::VectorXd task_p;
	Eigen::VectorXd task_pd;
	Eigen::VectorXd task_pdd;

};
#endif
