#include "rtt-controller-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <nemo/Vector.h>
#include <nemo/Mapping.h>
#include <rtt/Activity.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace rci;

#define l(lvl) log(lvl) << "[" << this->getName() << "] "

void handleIncomingRCIJointTorquesFromRSB(rci::JointAnglesPtr jTorques) {
	log(Info) << "whooooop: " << jTorques->print() << endlog();

	boost::mutex::scoped_lock lock(rsbcmdJointTorques_mtx);
	{
		rsbcmdJointTorques->setValues(jTorques->asDoubleVector());
	}

}

RTTController::RTTController(std::string const& name) :
		TaskContext(name),
		// Name, initial value
		cmdJntPos_Port("cmdJntPos", 0.0), cmdJntTrq_Port("cmdJntTrq", 0.0), currJntPos_Port(
				"currJntPos"), currJntVel_Port("currJntVel"), currJntTrq_Port(
				"currJntTrq"), gain(0.0),

		cmdJntTrq_Port_1("cmdJntTrq_1"), cmdJntTrq_Port_2("cmdJntTrq_2"), cmdJntTrq_Port_3(
				"cmdJntTrq_3"), cmdJntTrq_Port_4("cmdJntTrq_4"), cmdJntTrq_Port_5(
				"cmdJntTrq_5"), cmdJntTrq_Port_6("cmdJntTrq_6") {

	this->ports()->addPort(cmdJntPos_Port).doc(
			"Sending joint position commands.");
	this->ports()->addPort(cmdJntTrq_Port).doc(
			"Sending joint torque commands.");
	this->ports()->addPort(currJntPos_Port).doc(
			"Receiving current joint position.");
	this->ports()->addPort(currJntVel_Port).doc(
			"Receiving current joint velocity.");
	this->ports()->addPort(currJntTrq_Port).doc(
			"Receiving current joint torque.");

	this->ports()->addPort(cmdJntTrq_Port_1).doc(
			"Sending joint torque commands.");
	this->ports()->addPort(cmdJntTrq_Port_2).doc(
			"Sending joint torque commands.");
	this->ports()->addPort(cmdJntTrq_Port_3).doc(
			"Sending joint torque commands.");
	this->ports()->addPort(cmdJntTrq_Port_4).doc(
			"Sending joint torque commands.");
	this->ports()->addPort(cmdJntTrq_Port_5).doc(
			"Sending joint torque commands.");
	this->ports()->addPort(cmdJntTrq_Port_6).doc(
			"Sending joint torque commands.");

	// test TODO
	gain = 10.0;
	outJntPos = JointAngles::create(7, 0.0);
	cmdJntPos_Port.setDataSample(JointAngles::create(7, 0.0));

	outJntTrq = JointTorques::create(7, 0.0);
	rsbcmdJointTorques = outJntTrq;

	cmdJntTrq_Port.setDataSample(JointTorques::create(1, 0.0));
	cmdJntTrq_Port_1.setDataSample(JointTorques::create(1, 0.0));
	cmdJntTrq_Port_2.setDataSample(JointTorques::create(1, 0.0));
	cmdJntTrq_Port_3.setDataSample(JointTorques::create(1, 0.0));
	cmdJntTrq_Port_4.setDataSample(JointTorques::create(1, 0.0));
	cmdJntTrq_Port_5.setDataSample(JointTorques::create(1, 0.0));
	cmdJntTrq_Port_6.setDataSample(JointTorques::create(1, 0.0));

	// set up listener to receive cmds from outside
	rsb::Factory& factory = rsb::getFactory();
	rsb::Scope scope("/rtt/lwr/cmd/position");
	rsbcmdJointPosition_Listener = factory.createListener(scope);
	rsbcmdJointPosition_Listener->addHandler(
			rsb::HandlerPtr(
					new rsb::DataFunctionHandler<rci::JointAngles>(
							&handleIncomingRCIJointTorquesFromRSB)));

//	npactivity = Activity(12); // priority=12, no period.

	log(Info) << "RTTController constructed !" << endlog();
}

bool RTTController::configureHook() {
	log(Info) << "RTTController configured !" << endlog();
	return true;
}

bool RTTController::startHook() {
//	npactivity.run(&run_impl_1);
//	npactivity.start(); // calls 'loop()'
	log(Info) << "RTTController started !" << endlog();
	return true;

}

void RTTController::updateHook() {
	// to check for NewData, OldData or NoData use:
	// if (currJntPos_Port.read(currJntPos) == RTT::NewData) etc...
	if (currJntPos_Port.connected())
		currJntPos_Port.read(currJntPos);
	if (currJntVel_Port.connected())
		currJntVel_Port.read(currJntVel);
	if (currJntTrq_Port.connected())
		currJntTrq_Port.read(currJntTrq);

	/** INSERT TORQUE or POSITION CONTROLLER HERE */
	if (cmdJntTrq_Port.connected()) {
//		for (int i = 0; i < outJntTrq->NmVector().dimension(); i++) {
//			outJntTrq->setFromNm(i, gain);
////		RTT::log(RTT::Error) << i << " = currPos: " << currJntPos[i] << ", currVel: " << currJntVel[i] << ", currTrq: " << currJntTrq[i] << " ! ";
//		}

//		outJntTrq->setFromNm(1, gain);

		{
			boost::mutex::scoped_lock lock(rsbcmdJointTorques_mtx,
					boost::try_to_lock);
			if (lock) {
				outJntTrq = rsbcmdJointTorques;
			}
		}
		cmdJntTrq_Port.write(JointTorques::fromNm(outJntTrq->Nm(0)));
		cmdJntTrq_Port_1.write(JointTorques::fromNm(outJntTrq->Nm(1)));
		cmdJntTrq_Port_2.write(JointTorques::fromNm(outJntTrq->Nm(2)));
		cmdJntTrq_Port_3.write(JointTorques::fromNm(outJntTrq->Nm(3)));
		cmdJntTrq_Port_4.write(JointTorques::fromNm(outJntTrq->Nm(4)));
		cmdJntTrq_Port_5.write(JointTorques::fromNm(outJntTrq->Nm(5)));
		cmdJntTrq_Port_6.write(JointTorques::fromNm(outJntTrq->Nm(6)));

	}
	// you can also use cmdJntPos_Port.write(currJntPos); to control the position

}

void RTTController::stopHook() {
	log(Info) << "RTTController executes stopping !" << endlog();
}

void RTTController::cleanupHook() {
	log(Info) << "RTTController cleaning up !" << endlog();
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
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(RTTController)
//ORO_CREATE_COMPONENT(RTTController)
