#include <rtt/Component.hpp>


#include "rtt-QuinticPolynomial-wrapper.hpp"


#define l(lvl) log(lvl) << "[" << this->getName() << "] "

RttQuinticPolynomialWrapper::RttQuinticPolynomialWrapper(std::string const& name, double start_time, unsigned int numJoints) :
		TaskContext(name),
		start_time(start_time),
		numJoints(numJoints),
		cmdJntPos_Port("cmdJntPos", false),
		cmdJntVel_Port("cmdJntVel", false),
		cmdJntAcc_Port("cmdJntAcc", false)
	{

	//output ports
	this->ports()->addPort(cmdJntPos_Port).doc(
			"Sending jnt. position translation commands.");
	this->ports()->addPort(cmdJntVel_Port).doc(
			"Sending jnt. velocity translation commands.");
	this->ports()->addPort(cmdJntAcc_Port).doc(
			"Sending jnt. acceleration translation commands.");


	cmdJntPos.resize(numJoints);
	cmdJntVel.resize(numJoints);
	cmdJntAcc.resize(numJoints);

	l(Info) << "constructed !" << endlog();
}

bool RttQuinticPolynomialWrapper::configureHook() {
	Eigen::VectorXd init;
	Eigen::VectorXd final;
	init.resize(numJoints);
	final.resize(numJoints);
	init.setZero(numJoints);
	final.setOnes(numJoints);
//	final << 1.5708, 1.3963, 1.2217, 1.0472, 0.8727, 0.6981, 0.5236;
	this->jnt_task = QuinticPolynomial(numJoints, this->start_time, this->start_time+30, init, final);
	l(Info) << "configured !" << endlog();
	return true;
}

bool RttQuinticPolynomialWrapper::startHook() {
	this->start_time = getSimulationTime(); //TODO: correct??
	l(Info) << "started !" << endlog();
	return true;

}

double RttQuinticPolynomialWrapper::getSimulationTime() {
	return 1E-9
			* RTT::os::TimeService::ticks2nsecs(
					RTT::os::TimeService::Instance()->getTicks());
}

void RttQuinticPolynomialWrapper::updateHook() {
	//getting desired values FOR ENDEFFECTOR TRAJECOTRY:
	double t = getSimulationTime();
	double time_diff = t - this->start_time;
	jnt_task.getQ(time_diff, cmdJntPos);
	jnt_task.getQd(time_diff, cmdJntVel);
	jnt_task.getQdd(time_diff, cmdJntAcc);

	// send references via ports
	if (cmdJntPos_Port.connected()) {
		cmdJntPos_Port.write(cmdJntPos);
	}
	if (cmdJntVel_Port.connected()) {
		cmdJntVel_Port.write(cmdJntVel);
	}
	if (cmdJntAcc_Port.connected()) {
		cmdJntAcc_Port.write(cmdJntAcc);
	}
}

void RttQuinticPolynomialWrapper::stopHook() {
	l(Info) << "executes stopping !" << endlog();
}

void RttQuinticPolynomialWrapper::cleanupHook() {
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
ORO_LIST_COMPONENT_TYPE(RttQuinticPolynomialWrapper)
