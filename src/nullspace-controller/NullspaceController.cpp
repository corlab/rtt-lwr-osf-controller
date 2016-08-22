/* Author: Niels Dehio
 * Date:   16 August 2016
 *
 * Description: 
 */

#include "NullspaceController.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


NullspaceController::NullspaceController(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setDOFsize", &NullspaceController::setDOFsize, this).doc("set DOF size");
    addOperation("setGains", &NullspaceController::setGains, this).doc("set gains");
    addOperation("setDesiredAngles", &NullspaceController::setDesiredAngles, this).doc("set desired angles");
    addOperation("displayStatus", &NullspaceController::displayStatus, this).doc("print status");

    //other stuff
    gainP = 1;
    gainD = 0;
    receiveTranslationOnly = true;
    if(receiveTranslationOnly){
        TaskSpaceDimension = 3;
    }
    else{
        TaskSpaceDimension = 6;
    }
    portsArePrepared = false;
}

bool NullspaceController::configureHook() {
    return true;
}

bool NullspaceController::startHook() {
    if (!in_robotstatus_port.connected()) {
        RTT::log(RTT::Info) << "in_robotstatus_port not connected" << RTT::endlog();
        return false;
    }
    if (!in_jacobian_port.connected()) {
        RTT::log(RTT::Info) << "in_jacobian_port not connected" << RTT::endlog();
        return false;
    }
    if (!in_jacobianInv_port.connected()) {
        RTT::log(RTT::Info) << "in_jacobianInv_port not connected" << RTT::endlog();
        return false;
    }
    if (!out_torques_port.connected()) {
        RTT::log(RTT::Info) << "out_torques_port not connected" << RTT::endlog();
        return false;
    }
    return true;
}

void NullspaceController::updateHook() {
    //update desired angles only when new weight-vector is received
    if (in_desiredAngles_port.connected()) {
        in_desiredAngles_flow = in_desiredAngles_port.read(in_desiredAngles_var);
        if (in_desiredAngles_flow == RTT::NewData){
            current_desiredAngles.angles = in_desiredAngles_var.angles;
        }
    }

    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    in_jacobian_flow = in_jacobian_port.read(in_jacobian_var);
    in_jacobianInv_flow = in_jacobianInv_port.read(in_jacobianInv_var);

    if (in_jacobian_flow == RTT::NoData || in_jacobianInv_flow == RTT::NoData){
        return;
    }
    assert(in_jacobian_var.rows() == TaskSpaceDimension);
    assert(in_jacobian_var.cols() == DOFsize);
    assert(in_jacobianInv_var.rows() == TaskSpaceDimension);
    assert(in_jacobianInv_var.cols() == DOFsize);
    assert(current_desiredAngles.size() == DOFsize);

    out_torques_var.torques.setZero();

    //Eq. 13
    out_torques_var.torques = (identityDOFsizeDOFsize - in_jacobian_var.transpose() * in_jacobianInv_var);

    //Eq. 17
    out_torques_var.torques *= (gainP * (current_desiredAngles.angles - in_robotstatus_var.angles) - gainD * in_robotstatus_var.velocities );
    out_torques_port.write(out_torques_var);
}

void NullspaceController::stopHook() {
}

void NullspaceController::cleanupHook() {
    portsArePrepared = false;
}

void NullspaceController::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
    this->identityDOFsizeDOFsize = Eigen::MatrixXf(DOFsize,DOFsize);
    this->current_desiredAngles = rstrt::kinematics::JointAngles(DOFsize);
    this->preparePorts();
}

void NullspaceController::setGains(float kp, float kd){
    assert(kp>=0);
    assert(kd>=0);
    gainP = kp;
    gainD = kd;
}

bool NullspaceController::setDesiredAngles(rstrt::kinematics::JointAngles desiredAngles) {
    if(current_desiredAngles.angles.size() == DOFsize){
        this->current_desiredAngles.angles = desiredAngles.angles;
        return true;
    }
    else{
        return false;
    }
}

void NullspaceController::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("in_robotstatus_port");
        ports()->removePort("in_desiredAngles_port");
        ports()->removePort("in_jacobian_port");
        ports()->removePort("in_jacobianInv_port");
        ports()->removePort("out_torques_port");
    }

    //prepare input
    in_robotstatus_var = rstrt::robot::JointState(DOFsize);
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for reading robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    in_desiredAngles_var = rstrt::kinematics::JointAngles(DOFsize);
    in_desiredAngles_port.setName("in_desiredAngles_port");
    in_desiredAngles_port.doc("Input port for reading desired angles values");
    ports()->addPort(in_desiredAngles_port);
    in_desiredAngles_flow = RTT::NoData;

    in_jacobian_var = Eigen::MatrixXf(TaskSpaceDimension,DOFsize);
    in_jacobian_port.setName("in_jacobian_port");
    in_jacobian_port.doc("Input port for reading jacobian values");
    ports()->addPort(in_jacobian_port);
    in_jacobian_flow = RTT::NoData;

    in_jacobianInv_var = Eigen::MatrixXf(TaskSpaceDimension,DOFsize);
    in_jacobianInv_port.setName("in_jacobianInv_port");
    in_jacobianInv_port.doc("Input port for reading jacobian special inverse values");
    ports()->addPort(in_jacobianInv_port);
    in_jacobianInv_flow = RTT::NoData;

    //prepare output
    out_torques_var = rstrt::dynamics::JointTorques(DOFsize);
    out_torques_var.torques.setZero();
    out_torques_port.setName("out_torques_port");
    out_torques_port.doc("Output port for sending torque values");
    out_torques_port.setDataSample(out_torques_var);
    ports()->addPort(out_torques_port);

    portsArePrepared = true;
}

void NullspaceController::displayStatus(){
    RTT::log(RTT::Info) << "in_desiredAngles_var \n" << in_desiredAngles_var << RTT::endlog();
    RTT::log(RTT::Info) << "in_jacobian_var \n" << in_jacobian_var << RTT::endlog();
    RTT::log(RTT::Info) << "in_jacobianInv_var \n" << in_jacobianInv_var << RTT::endlog();
    RTT::log(RTT::Info) << "out_torques_var \n" << out_torques_var.torques << RTT::endlog();

    RTT::log(RTT::Info) << "deviation angles \n" << in_desiredAngles_var.angles - in_robotstatus_var.angles << RTT::endlog();
    RTT::log(RTT::Info) << "gainP \n" << gainP << RTT::endlog();
    RTT::log(RTT::Info) << "gainD \n" << gainD << RTT::endlog();
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(NullspaceController)
