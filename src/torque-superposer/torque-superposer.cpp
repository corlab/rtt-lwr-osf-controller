#include "torque-superposer.hpp"

#include <rtt/Component.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

TorqueSuperposer::TorqueSuperposer(std::string const& name) :
		RTT::TaskContext(name), numberOfPorts(2), dimensionOfInput(7), trq_out_port(
				"output"), fixedNumPorts(2) {

	this->addProperty("numberOfPorts", numberOfPorts);
	this->addProperty("dimensionOfInput", dimensionOfInput);
}

bool TorqueSuperposer::configureHook() {
	fixedNumPorts = numberOfPorts;
	trqCmdOutput = rci::JointTorques::create(dimensionOfInput, 0.0);
	tmpTrqValue = rci::JointTorques::create(dimensionOfInput, 0.0);

	// set output port
	trq_out_port.setDataSample(trqCmdOutput);
	this->ports()->addPort(trq_out_port);

	// set input port
	for (int i = 0; i < fixedNumPorts; i++) {
		boost::shared_ptr<RTT::InputPort<rci::JointTorquesPtr> > tmpPort(
				new RTT::InputPort<rci::JointTorquesPtr>(
						"input_" + boost::lexical_cast<std::string>(i)));
		RTT::FlowStatus tmpFlow(RTT::NoData);
		trq_in_ports.push_back(tmpPort);
		trq_in_flows.push_back(tmpFlow);
		weights.push_back(0.0);
	}
	return true;
}

bool TorqueSuperposer::startHook() {
	return true;
}

void TorqueSuperposer::updateHook() {
	for (int i = 0; i < fixedNumPorts; i++) {
		if (trq_in_ports[i]->read(tmpTrqValue) != RTT::NoData) {
			trqCmdOutput->setValues(
					trqCmdOutput->asDoubleVector()
							+ trqCmdOutput->asDoubleVector() * weights[i]);
		}
	}

	if (trq_out_port.connected()) {
		trq_out_port.write(trqCmdOutput);
	}
}

void TorqueSuperposer::stopHook() {

}

void TorqueSuperposer::cleanupHook() {
	this->ports()->clear();
}

ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(TorqueSuperposer)
