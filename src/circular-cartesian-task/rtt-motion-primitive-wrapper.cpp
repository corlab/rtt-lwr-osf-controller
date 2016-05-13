#include <rtt/Component.hpp>


#include "rtt-motion-primitive-wrapper.hpp"


#define l(lvl) log(lvl) << "[" << this->getName() << "] "

RttMotionPrimitiveWrapper::RttMotionPrimitiveWrapper(std::string const& name, double start_time) :
		TaskContext(name),
		start_time(start_time),
		cmdCartPosTranslation_Port("cmdCartPosTranslation", false),
		cmdCartVelTranslation_Port("cmdCartVelTranslation", false),
		cmdCartAccTranslation_Port("cmdCartAccTranslation", false),
		cmdCartPosOrientation_Port("cmdCartPosOrientation", false),
		cmdCartVelOrientation_Port("cmdCartVelOrientation", false),
		cmdCartAccOrientation_Port("cmdCartAccOrientation", false)
	{

	//output translation ports
	this->ports()->addPort(cmdCartPosTranslation_Port).doc(
			"Sending cart. position translation commands.");
	this->ports()->addPort(cmdCartVelTranslation_Port).doc(
			"Sending cart. velocity translation commands.");
	this->ports()->addPort(cmdCartAccTranslation_Port).doc(
			"Sending cart. acceleration translation commands.");

	//output orientation ports
	this->ports()->addPort(cmdCartPosOrientation_Port).doc(
			"Sending cart. position orientation commands.");
	this->ports()->addPort(cmdCartVelOrientation_Port).doc(
			"Sending cart. velocity orientation commands.");
	this->ports()->addPort(cmdCartAccOrientation_Port).doc(
			"Sending cart. acceleration orientation commands.");


	cmdCartPosTranslation.resize(3);
	cmdCartVelTranslation.resize(3);
	cmdCartAccTranslation.resize(3);
	cmdCartPosOrientation.resize(3);
	cmdCartVelOrientation.resize(3);
	cmdCartAccOrientation.resize(3);

	l(Info) << "constructed !" << endlog();
}

bool RttMotionPrimitiveWrapper::configureHook() {
	this->cart_task = CartesianSpace_CircularTask(this->start_time, 0.6);
	l(Info) << "configured !" << endlog();
	return true;
}

bool RttMotionPrimitiveWrapper::startHook() {
	this->start_time = getSimulationTime(); //TODO: correct??
	l(Info) << "started !" << endlog();
	return true;

}

double RttMotionPrimitiveWrapper::getSimulationTime() {
	return 1E-9
			* RTT::os::TimeService::ticks2nsecs(
					RTT::os::TimeService::Instance()->getTicks());
}

void RttMotionPrimitiveWrapper::updateHook() {
	//getting desired values FOR ENDEFFECTOR TRAJECOTRY:
	double t = getSimulationTime();
	double time_diff = t - this->start_time;
	cart_task.getPositionTranslation(time_diff, cmdCartPosTranslation);
	cart_task.getVelocityTranslation(time_diff, cmdCartVelTranslation);
	cart_task.getAccelerationTranslation(time_diff, cmdCartAccTranslation);

	cart_task.getPositionOrientation(time_diff, cmdCartPosOrientation);
	cart_task.getVelocityOrientation(time_diff, cmdCartVelOrientation);
	cart_task.getAccelerationOrientation(time_diff, cmdCartAccOrientation);

	// send references via translation ports
	if (cmdCartPosTranslation_Port.connected()) {
		cmdCartPosTranslation_Port.write(cmdCartPosTranslation);
	}
	if (cmdCartVelTranslation_Port.connected()) {
		cmdCartVelTranslation_Port.write(cmdCartVelTranslation);
	}
	if (cmdCartAccTranslation_Port.connected()) {
		cmdCartAccTranslation_Port.write(cmdCartAccTranslation);
	}

	// send references via orientation ports
	if (cmdCartPosOrientation_Port.connected()) {
		cmdCartPosOrientation_Port.write(cmdCartPosOrientation);
	}
	if (cmdCartVelOrientation_Port.connected()) {
		cmdCartVelOrientation_Port.write(cmdCartVelOrientation);
	}
	if (cmdCartAccOrientation_Port.connected()) {
		cmdCartAccOrientation_Port.write(cmdCartAccOrientation);
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
