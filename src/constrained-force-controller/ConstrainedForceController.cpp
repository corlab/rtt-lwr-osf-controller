/* Author: Niels Dehio
 * Date:   16 August 2016
 *
 * Description: 
 */

#include "ConstrainedForceController.hpp"

#include <rtt/Component.hpp> // needed for the macro at the end of this file

ConstrainedForceController::ConstrainedForceController(std::string const & name) :
        RTT::TaskContext(name) {
	//prepare operations
	addOperation("setDOFsize", &ConstrainedForceController::setDOFsize, this).doc(
			"set DOF size");
    addOperation("setLambda", &ConstrainedForceController::setLambda, this).doc(
            "set lambda");
    addOperation("displayStatus", &ConstrainedForceController::displayStatus, this).doc("print status");

    addOperation("preparePorts", &ConstrainedForceController::preparePorts, this).doc(
        				"preparePorts");
    addOperation("setCstrSpaceDimension", &ConstrainedForceController::setCstrSpaceDimension,
            this, RTT::ClientThread).doc(
            "set CstrSpaceDimension");

    current_lambda = Eigen::VectorXf::Zero(CstrSpaceDimension);
	portsArePrepared = false;
}

bool ConstrainedForceController::configureHook() {
	return true;
}

bool ConstrainedForceController::startHook() {
    if (!in_robotstatus_port.connected()) {
        RTT::log(RTT::Info) << "in_robotstatus_port not connected" << RTT::endlog();
        return false;
    }
    if (!in_jacobian_c_port.connected()) {
        RTT::log(RTT::Info) << "in_jacobian_c_port not connected" << RTT::endlog();
        return false;
    }
    if (!in_inertia_port.connected()) {
        RTT::log(RTT::Info) << "in_inertia_port not connected" << RTT::endlog();
        return false;
    }
    if (!in_inertia_c_port.connected()) {
        RTT::log(RTT::Info) << "in_inertia_c_port not connected" << RTT::endlog();
        return false;
    }
    if (!in_p_port.connected()) {
        RTT::log(RTT::Info) << "in_p_port not connected" << RTT::endlog();
        return false;
    }
    if (!in_h_port.connected()) {
        RTT::log(RTT::Info) << "in_h_port not connected" << RTT::endlog();
        return false;
    }
    if (!in_Cc_port.connected()) {
        RTT::log(RTT::Info) << "in_Cc_port not connected" << RTT::endlog();
        return false;
    }
    if (!out_torques_port.connected()) {
        RTT::log(RTT::Info) << "out_torques_port not connected" << RTT::endlog();
        return false;
    }
    //TODO add more
	return true;
}

void ConstrainedForceController::updateHook() {
    //update desired lambda only when new lambda-vector is received
	if (in_lambda_des_port.connected()) {
		in_lambda_des_flow = in_lambda_des_port.read(in_lambda_des_var);
        if (in_lambda_des_flow == RTT::NewData){
            current_lambda = in_lambda_des_var;
        }
	}

	in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
	in_jacobian_c_flow = in_jacobian_c_port.read(in_jacobian_c_var);
	in_inertia_flow = in_inertia_port.read(in_inertia_var);
	in_inertia_c_flow = in_inertia_c_port.read(in_inertia_c_var);
	in_p_flow = in_p_port.read(in_p_var);
	in_h_flow = in_h_port.read(in_h_var);
	in_Cc_flow = in_Cc_port.read(in_Cc_var);

	if (in_robotstatus_flow == RTT::NoData || in_jacobian_c_flow == RTT::NoData
			|| in_inertia_flow == RTT::NoData
			|| in_inertia_c_flow == RTT::NoData || in_p_flow == RTT::NoData
			|| in_h_flow == RTT::NoData || in_Cc_flow == RTT::NoData) {
		return;
	}

	out_torques_var.torques.setZero();

//    Eigen::VectorXf foo;
//    foo = Eigen::VectorXf::Zero(14);
//    foo = in_robotstatus_var.velocities;
//    foo(0) = in_robotstatus_var.velocities(6);//reverse order of left arm joint values
//    foo(1) = in_robotstatus_var.velocities(5);
//    foo(2) = in_robotstatus_var.velocities(4);
//    foo(3) = in_robotstatus_var.velocities(3);
//    foo(4) = in_robotstatus_var.velocities(2);
//    foo(5) = in_robotstatus_var.velocities(1);
//    foo(6) = in_robotstatus_var.velocities(0);
//    in_robotstatus_var.velocities = foo;

    out_torques_var.torques = in_h_var + in_inertia_var * (in_inertia_c_var+0.00001*identityDOFsizeDOFsize).inverse()
                    * (in_p_var * in_robotstatus_var.torques - in_p_var * in_h_var + in_Cc_var * in_robotstatus_var.velocities)
            + (in_jacobian_c_var.transpose() * current_lambda);

//    Eigen::VectorXf bar;
//    bar = Eigen::VectorXf::Zero(14);
//    bar = out_torques_var.torques;
//    bar(0) = out_torques_var.torques(6);//reverse order of left arm joint values
//    bar(1) = out_torques_var.torques(5);
//    bar(2) = out_torques_var.torques(4);
//    bar(3) = out_torques_var.torques(3);
//    bar(4) = out_torques_var.torques(2);
//    bar(5) = out_torques_var.torques(1);
//    bar(6) = out_torques_var.torques(0);
//    out_torques_var.torques = bar;

	out_torques_port.write(out_torques_var);
}

void ConstrainedForceController::stopHook() {
}

void ConstrainedForceController::cleanupHook() {
	portsArePrepared = false;
}

void ConstrainedForceController::setDOFsize(unsigned int DOFsize) {
	assert(DOFsize > 0);
	this->DOFsize = DOFsize;
    this->identityDOFsizeDOFsize = Eigen::MatrixXf::Identity(DOFsize,DOFsize);
}

void ConstrainedForceController::setCstrSpaceDimension(const unsigned int CstrSpaceDimension) {
    this->CstrSpaceDimension = CstrSpaceDimension;
}

void ConstrainedForceController::setLambda(Eigen::VectorXf new_lambda) {
    assert(new_lambda.size() == CstrSpaceDimension);
    this->current_lambda = new_lambda;
}

void ConstrainedForceController::preparePorts() {
	if (portsArePrepared) {
		ports()->removePort("in_robotstatus_port");
		ports()->removePort("in_lambda_des_port");
		ports()->removePort("in_jacobian_c_port");
		ports()->removePort("in_inertia_port");
		ports()->removePort("in_inertia_c_port");
		ports()->removePort("in_p_port");
		ports()->removePort("in_h_port");
		ports()->removePort("in_Cc_port");
	}

	//prepare input
	in_robotstatus_var = rstrt::robot::JointState(DOFsize);
	in_robotstatus_port.setName("in_robotstatus_port");
	in_robotstatus_port.doc("Input port for reading robotstatus values");
	ports()->addPort(in_robotstatus_port);
	in_robotstatus_flow = RTT::NoData;

    in_lambda_des_var = Eigen::VectorXf(CstrSpaceDimension);
	in_lambda_des_port.setName("in_lambda_des_port");
	in_lambda_des_port.doc("Input port for reading desired lambdas");
	ports()->addPort(in_lambda_des_port);
	in_lambda_des_flow = RTT::NoData;

    in_jacobian_c_var = Eigen::MatrixXf(CstrSpaceDimension, DOFsize);
    in_jacobian_c_port.setName("in_jacobian_c_port");
	in_jacobian_c_port.doc("Input port for reading jacobian_c values");
	ports()->addPort(in_jacobian_c_port);
	in_jacobian_c_flow = RTT::NoData;

	in_inertia_var = Eigen::MatrixXf(DOFsize, DOFsize);
	in_inertia_port.setName("in_inertia_port");
	in_inertia_port.doc("Input port for reading inertia values");
	ports()->addPort(in_inertia_port);
	in_inertia_flow = RTT::NoData;

	in_inertia_c_var = Eigen::MatrixXf(DOFsize, DOFsize);
	in_inertia_c_port.setName("in_inertia_c_port");
	in_inertia_c_port.doc("Input port for reading inertia_c values");
	ports()->addPort(in_inertia_c_port);
	in_inertia_c_flow = RTT::NoData;

    in_p_var = Eigen::MatrixXf(DOFsize, DOFsize);
	in_p_port.setName("in_p_port");
	in_p_port.doc("Input port for reading p values");
	ports()->addPort(in_p_port);
	in_p_flow = RTT::NoData;

	in_h_var = Eigen::VectorXf(DOFsize);
	in_h_port.setName("in_h_port");
	in_h_port.doc("Input port for reading h values");
	ports()->addPort(in_h_port);
	in_h_flow = RTT::NoData;

	in_Cc_var = Eigen::MatrixXf(DOFsize, DOFsize);
	in_Cc_port.setName("in_Cc_port");
	in_Cc_port.doc("Input port for reading Cc values");
	ports()->addPort(in_Cc_port);
	in_Cc_flow = RTT::NoData;

	//prepare output
	out_torques_var = rstrt::dynamics::JointTorques(DOFsize);
	out_torques_var.torques.setZero();
	out_torques_port.setName("out_torques_port");
	out_torques_port.doc("Output port for sending torque values");
	out_torques_port.setDataSample(out_torques_var);
	ports()->addPort(out_torques_port);

	portsArePrepared = true;
}

void ConstrainedForceController::displayStatus(){
    RTT::log(RTT::Info) << "in_robotstatus_var.torques \n" << in_robotstatus_var.torques << RTT::endlog();
    RTT::log(RTT::Info) << "in_lambda_des_var \n" << in_lambda_des_var << RTT::endlog();
    RTT::log(RTT::Info) << "in_jacobian_c_var \n" << in_jacobian_c_var << RTT::endlog();
    RTT::log(RTT::Info) << "in_inertia_var \n" << in_inertia_var << RTT::endlog();
    RTT::log(RTT::Info) << "in_inertia_c_var \n" << in_inertia_c_var << RTT::endlog();
    RTT::log(RTT::Info) << "in_p_var \n" << in_p_var << RTT::endlog();
    RTT::log(RTT::Info) << "in_h_var \n" << in_h_var << RTT::endlog();
    RTT::log(RTT::Info) << "in_Cc_var \n" << in_Cc_var << RTT::endlog();
    RTT::log(RTT::Info) << "out_torques_var \n" << out_torques_var.torques << RTT::endlog();
    RTT::log(RTT::Info) << "current_lambda \n" << current_lambda << RTT::endlog();
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(ConstrainedForceController)
