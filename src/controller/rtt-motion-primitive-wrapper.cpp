#include <rtt/Component.hpp>
#include <iostream>
#include <nemo/Vector.h>
#include <nemo/Mapping.h>
#include <rtt/Activity.hpp>
#include "rtt-motion-primitive-wrapper.hpp"

#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace rci;

#define l(lvl) log(lvl) << "[" << this->getName() << "] "

RttMotionPrimitiveWrapper::RttMotionPrimitiveWrapper(std::string const& name) :
		TaskContext(name), start_time(0.0), cmdCartPos_Port("cmdCartPos",
				false), cmdCartVel_Port("cmdCartVel", false), cmdCartAcc_Port(
				"cmdCartAcc", false) {

	this->ports()->addPort(cmdCartPos_Port).doc(
			"Sending cart. position commands.");
	this->ports()->addPort(cmdCartVel_Port).doc(
			"Sending cart. velocity commands.");
	this->ports()->addPort(cmdCartAcc_Port).doc(
			"Sending cart. acceleration commands.");

	cmdCartPos = rci::Pose::fromMetersAndRadians(0, 0, 0, 0, 0, 0, 0);
	cmdCartVel = rci::Pose::fromMetersAndRadians(0, 0, 0, 0, 0, 0, 0);
	cmdCartAcc = rci::Pose::fromMetersAndRadians(0, 0, 0, 0, 0, 0, 0);

	l(Info) << "constructed !" << endlog();
}

bool RttMotionPrimitiveWrapper::configureHook() {
	this->cart_task = CartesianSpace_CircularTask(0.0, 0.6);
	l(Info) << "configured !" << endlog();
	return true;
}

bool RttMotionPrimitiveWrapper::startHook() {
	this->start_time = getSimulationTime();
	l(Info) << "started !" << endlog();
	return true;

}

double RttMotionPrimitiveWrapper::getSimulationTime() {
	return 1E-9
			* RTT::os::TimeService::ticks2nsecs(
					RTT::os::TimeService::Instance()->getTicks());
}

void RttMotionPrimitiveWrapper::updateHook() {

	double t = getSimulationTime();

	//getting desired values FOR ENDEFFECTOR TRAJECOTRY:
	cart_task.getPosition(t - this->start_time, task_p);
	cart_task.getVelocity(t - this->start_time, task_pd);
	cart_task.getAcceleration(t - this->start_time, task_pdd);

	//TODO convert to quaternion and set the datatypes differently,
	// cause this way i am not sure if new memory gets allocated each time...

	// convert eigen back to rci
	cmdCartPos = rci::Pose::fromMetersAndRadians((double) task_p(0),
			(double) task_p(1), (double) task_p(2), (double) task_p(3),
			(double) task_p(4), (double) task_p(5));
	cmdCartVel = rci::Pose::fromMetersAndRadians((double) task_pd(0),
			(double) task_pd(1), (double) task_pd(2), (double) task_pd(3),
			(double) task_pd(4), (double) task_pd(5));
	cmdCartAcc = rci::Pose::fromMetersAndRadians((double) task_pdd(0),
			(double) task_pdd(1), (double) task_pdd(2), (double) task_pdd(3),
			(double) task_pdd(4), (double) task_pdd(5));

	// write torques to robot
	if (cmdCartPos_Port.connected()) {
		cmdCartPos_Port.write(cmdCartPos);
	}
	if (cmdCartVel_Port.connected()) {
		cmdCartVel_Port.write(cmdCartVel);
	}
	if (cmdCartAcc_Port.connected()) {
		cmdCartAcc_Port.write(cmdCartAcc);
	}
}

void RttMotionPrimitiveWrapper::stopHook() {
	l(Info) << "executes stopping !" << endlog();
}

void RttMotionPrimitiveWrapper::cleanupHook() {
	l(Info) << "cleaning up !" << endlog();
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(RTTController)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_LIST_COMPONENT_TYPE(RttMotionPrimitiveWrapper)
