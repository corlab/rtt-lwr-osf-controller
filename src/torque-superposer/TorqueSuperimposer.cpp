/* Author: Niels Dehio
 * Date:   16 August 2016
 *
 * Description: 
 */

#include "TorqueSuperimposer.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


TorqueSuperimposer::TorqueSuperimposer(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setDOFsize", &TorqueSuperimposer::setDOFsize, this).doc("set DOF size");
    addOperation("setInitialWeights", &TorqueSuperimposer::setInitialWeights, this).doc("set initial weights");
    addOperation("preparePorts", &TorqueSuperimposer::preparePorts, this).doc("prepare ports");

    //other stuff
    portsArePrepared = false;
}

bool TorqueSuperimposer::configureHook() {
    return true;
}

bool TorqueSuperimposer::startHook() {
    return true;
}

void TorqueSuperimposer::updateHook() {
    //update weights only when new weight-vector is received
    if (in_weights_port.connected()) {
        in_weights_flow = in_weights_port.read(in_weights_var);
        if (in_weights_flow == RTT::NewData){
            current_weights = in_weights_var;
        }
    }

    in_projection_flow = in_projection_port.read(in_projection_var);
    in_torquesA_flow = in_torquesA_port.read(in_torquesA_var);
    in_torquesB_flow = in_torquesB_port.read(in_torquesB_var);
    in_torquesC_flow = in_torquesC_port.read(in_torquesC_var);

    out_torques_var.torques.setZero();
    out_torques_var.torques += current_weights.weights(0) * in_projection_var * in_torquesA_var.torques;
    out_torques_var.torques += current_weights.weights(1) * in_projection_var * in_torquesB_var.torques;
    out_torques_var.torques += current_weights.weights(2) * (identityDOFsizeDOFsize - in_projection_var) * in_torquesC_var.torques;
    out_torques_port.write(out_torques_var);
}

void TorqueSuperimposer::stopHook() {
}

void TorqueSuperimposer::cleanupHook() {
    portsArePrepared = false;
}

void TorqueSuperimposer::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
    this->current_weights = rstrt::robot::Weights(3);
    this->current_weights.weights.setOnes();
    this->identityDOFsizeDOFsize = Eigen::MatrixXf(DOFsize,DOFsize);
    this->preparePorts();
}

bool TorqueSuperimposer::setInitialWeights(rstrt::robot::Weights & initialWeights) {
    if (initialWeights.weights.size() == 3){
        current_weights = initialWeights;
        return true;
    }
    else{
        return false;
    }
}

void TorqueSuperimposer::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("in_weights_port");
        ports()->removePort("in_projection_port");
        ports()->removePort("in_torquesA_port");
        ports()->removePort("in_torquesB_port");
        ports()->removePort("in_torquesC_port");
        ports()->removePort("out_torques_port");
    }

    //prepare input
    in_weights_var = rstrt::robot::Weights(3);
    in_weights_port.setName("in_weights_port");
    in_weights_port.doc("Input port for reading weight values");
    ports()->addPort(in_weights_port);
    in_weights_flow = RTT::NoData;

    in_projection_var = Eigen::MatrixXf(DOFsize,DOFsize);
    in_projection_port.setName("in_projection_port");
    in_projection_port.doc("Input port for reading projection values");
    ports()->addPort(in_projection_port);
    in_projection_flow = RTT::NoData;

    in_torquesA_var = rstrt::dynamics::JointTorques(DOFsize);
    in_torquesA_port.setName("in_torquesA_port");
    in_torquesA_port.doc("Input port for reading torque values");
    ports()->addPort(in_torquesA_port);
    in_torquesA_flow = RTT::NoData;

    in_torquesB_var = rstrt::dynamics::JointTorques(DOFsize);
    in_torquesB_port.setName("in_torquesB_port");
    in_torquesB_port.doc("Input port for reading torque values");
    ports()->addPort(in_torquesB_port);
    in_torquesB_flow = RTT::NoData;

    in_torquesC_var = rstrt::dynamics::JointTorques(DOFsize);
    in_torquesC_port.setName("in_torquesC_port");
    in_torquesC_port.doc("Input port for reading torque values");
    ports()->addPort(in_torquesC_port);
    in_torquesC_flow = RTT::NoData;

    //prepare output
    out_torques_var = rstrt::dynamics::JointTorques(DOFsize);
    out_torques_var.torques.setZero();
    out_torques_port.setName("out_torques_port");
    out_torques_port.doc("Output port for sending torque values");
    out_torques_port.setDataSample(out_torques_var);
    ports()->addPort(out_torques_port);

    portsArePrepared = true;
}


//this macro should appear only once per library
ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(TorqueSuperimposer)

