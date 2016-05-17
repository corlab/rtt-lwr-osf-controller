#include <rtt/Component.hpp>


#include "rtt-jpc.hpp"


RTTJointPositionController::RTTJointPositionController(std::string const& name, unsigned int numJoints) :
    RTT::TaskContext(name),
    currJntPos_Flow(RTT::NoData),
    currJntVel_Flow(RTT::NoData),
    numJoints(numJoints)
    {
    //output joint torque port
    this->addPort("cmdJntTrq_", cmdJntTrq_Port);
    cmdJntTrq_ = rci::JointTorques::create(numJoints, 0.0);
    cmdJntTrq_Port.setDataSample(cmdJntTrq_);

    //input joint ports
    this->addPort("currJntPos", currJntPos_Port);
    this->addPort("currJntVel", currJntVel_Port);
    this->addPort("refJntPos", refJntPos_Port);
    this->addPort("refJntVel", refJntVel_Port).doc("Reference Joint Velocity Port");

}

bool RTTJointPositionController::configureHook() {
    if (!(cmdJntTrq_Port.connected() && currJntPos_Port.connected() && currJntVel_Port.connected() && refJntPos_Port.connected() && refJntVel_Port.connected())) {
        return false;
    }
    return true;
}

bool RTTJointPositionController::startHook() {

    //read parameters from config-file
    addProperty( "Kp_StiffnessGain", Kp_StiffnessGain ).doc("Kp_StiffnessGain");
    addProperty( "Kd_DampingGain", Kd_DampingGain ).doc("Kd_DampingGain");

    //fill gain matrices (should be added to the ops file!!!)
    Kp_StiffnessGain.setConstant(Kp_StiffnessGain);
    Kd_DampingGain.setConstant(Kd_DampingGain);
    Kp.resize(numJoints);
    Kp.setConstant(Kp_StiffnessGain);
    Kd.resize(numJoints);

    cmdJntTrq.resize(numJoints);

    //dynamic properpies
    G_.resize(numJoints);

    return true;
}

void RTTJointPositionController::updateHook() {

    //read input joint ports
    //currJntPos_Flow = currJntPos_Port.read(currJntPositionFB);
    currJntPos_Flow = currJntPos_Port.read(currJntPos);
    currJntVel_Flow = currJntVel_Port.read(currJntVel);
    refJntPos_Flow = refJntPos_Port.read(refJntVel);
    refJntVel_Flow = refJntVel_Port.read(refJntVel);

    //if (currJntPos_Flow == RTT::NewData) {
    //
    //}

    // start simple joint position controller
    cmdJntTrq = G_.data + Kp.asDiagonal() * (refJntPos - currJntPos) + Kd.asDiagonal() * (refJntVel - currJntVel);

    // convert eigen back to rci
    for (int i = 0; i < numJoints; i++) {
        cmdJntTrq_->setFromNm(i, cmdJntTrq.data()[i]);
    }

    if (cmdJntTrq_Port.connected()) {
        cmdJntTrq_Port.write(cmdJntTrq_);
    }
}

void RTTJointPositionController::stopHook() {

}

void RTTJointPositionController::cleanupHook() {

}


ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(RTTJointPositionController)
