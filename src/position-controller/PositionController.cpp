/* Author: Milad Malekzadeh
 * Date:   16 August 2016
 *
 * Description:
 */

#include "PositionController.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


PositionController::PositionController(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setDOFsize", &PositionController::setDOFsize, this).doc("set DOF size");

    //other stuff
    gainP_position = 100;
    gainD_position = 20;
    gainP_orientation = 100;
    gainD_orientation = 20;

    portsArePrepared = false;
}

bool PositionController::configureHook() {
    return true;
}

bool PositionController::startHook() {
    return true;
}

void PositionController::updateHook() {
    //update desired angles only when new weight-vector is received

    // to be done???

    //if (in_desiredAngles_port.connected()) {
    //    in_desiredAngles_flow = in_desiredAngles_port.read(in_desiredAngles_var);
    //    if (in_desiredAngles_flow == RTT::NewData){
    //        current_desiredAngles = in_desiredAngles_var;
    //    }
    //}

    // position
    in_desiredPos_flow = in_desiredPos_port.read(in_desiredPos_var);
    in_desiredPosVelocity_flow = in_desiredPosVelocity_port.read(in_desiredPosVelocity_var);
    in_desiredPosAcceleration_flow = in_desiredPosAcceleration_port.read(in_desiredPosAcceleration_var);

    in_currentPos_flow = in_currentPos_port.read(in_currentPos_var);
    in_currentPosVelocity_flow = in_currentPosVelocity_port.read(in_currentPosVelocity_var);
    in_currentJntVelocity_flow = in_currentJntVelocity_port.read(in_currentJntVelocity_var);

    // orientation
    in_desiredOrt_flow = in_desiredOrt_port.read(in_desiredOrt_var);
    in_desiredOrtVelocity_flow = in_desiredOrtVelocity_port.read(in_desiredOrtVelocity_var);
    in_desiredOrtAcceleration_flow = in_desiredOrtAcceleration_port.read(in_desiredOrtAcceleration_var);

    in_currentOrt_flow = in_currentOrt_port.read(in_currentOrt_var);
    in_currentOrtVelocity_flow = in_currentOrtVelocity_port.read(in_currentOrtVelocity_var);


    in_jacobian_flow = in_jacobian_port.read(in_jacobian_var);
    in_jacobianDot_flow = in_jacobianDot_port.read(in_jacobianDot_var);
    in_constraintLambda_flow = in_constraintLambda_port.read(in_constraintLambda_var);
    in_h_flow = in_h_port.read(in_h_var);
    in_constraintM_flow = in_constraintM_port.read(in_constraintM_var);
    in_P_flow = in_P_port.read(in_P_var);
    in_constraintC_flow = in_constraintC_port.read(in_constraintC_var);

    // reference acceleration for position
    ref_PosAcceleration = in_desiredPosAcceleration_var + gainP_position * (in_desiredPos_var - in_currentPos_var) + gainD_position * (in_desiredPosVelocity_var - in_currentPosVelocity_var);

    // reference acceleration for orientation
    ref_OrtAcceleration = in_desiredOrtAcceleration_var + gainP_orientation * (in_desiredOrt_var - in_currentOrt_var) + gainD_orientation * (in_desiredOrtVelocity_var - in_currentOrtVelocity_var);

    // reference accelerartion for orientation
    ref_Acceleration(0) = ref_PosAcceleration(0);
    ref_Acceleration(1) = ref_PosAcceleration(1);
    ref_Acceleration(2) = ref_PosAcceleration(2);
    ref_Acceleration(3) = ref_OrtAcceleration(0);
    ref_Acceleration(4) = ref_OrtAcceleration(1);
    ref_Acceleration(5) = ref_OrtAcceleration(2);

    //Start Khatib projected endeffector motion controller
    constraintForce = in_constraintLambda_var * ref_Acceleration;
    constraintForce += in_constraintLambda_var * ( in_jacobian_var * in_constraintM_var.inverse() * in_P_var * in_h_var - (in_jacobianDot_var + in_jacobian_var * in_constraintM_var.inverse() * in_constraintC_var ) * in_currentJntVelocity_var.velocities);

    out_torques_var.torques.setZero();
    out_torques_var.torques = in_jacobian_var.transpose() * constraintForce;
    out_torques_port.write(out_torques_var);
}

void PositionController::stopHook() {
}

void PositionController::cleanupHook() {
    portsArePrepared = false;
}

void PositionController::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
    ref_OrtAcceleration = Eigen::VectorXf::Zero(3);
    ref_Acceleration = Eigen::VectorXf::Zero(6);
    this->preparePorts();
}

void PositionController::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("in_desiredPos_port");  //1
        ports()->removePort("in_desiredPosVelocity_port");  //2
        ports()->removePort("in_desiredPosAcceleration_port");  //3

        ports()->removePort("in_currentPos_port");  //4
        ports()->removePort("in_currentPosVelocity_port");  //5
        ports()->removePort("in_currentJntVelocity_port");  //6

        ports()->removePort("in_desiredOrt_port");  //7
        ports()->removePort("in_desiredOrtVelocity_port");  //8
        ports()->removePort("in_desiredOrtAcceleration_port");  //9

        ports()->removePort("in_currentOrt_port");  //10
        ports()->removePort("in_currentOrtVelocity_port");  //11

        ports()->removePort("in_jacobian_port");    //12
        ports()->removePort("in_jacobianDot_port");     //13
        ports()->removePort("in_constraintLambda_port");    //14
        ports()->removePort("in_h_port");   //15
        ports()->removePort("in_constraintM_port");     //16
        ports()->removePort("in_P_port");   //17
        ports()->removePort("in_constraintC_port");     //18

        ports()->removePort("out_torques_port");

    }

    //prepare input
    //1
    in_desiredPos_var = Eigen::VectorXf(3);
    in_desiredPos_port.setName("in_desiredPos_port");
    in_desiredPos_port.doc("Input port for reading the desired position from Trajectory Generator");
    ports()->addPort(in_desiredPos_port);
    in_desiredPos_flow = RTT::NoData;
    //2
    in_desiredPosVelocity_var = Eigen::VectorXf(3);
    in_desiredPosVelocity_port.setName("in_desiredPosVelocity_port");
    in_desiredPosVelocity_port.doc("Input port for reading the desired velocity from Trajectory Generator");
    ports()->addPort(in_desiredPosVelocity_port);
    in_desiredPosVelocity_flow = RTT::NoData;
    //3
    in_desiredPosAcceleration_var = Eigen::VectorXf(3);
    in_desiredPosAcceleration_port.setName("in_desiredPosAcceleration_port");
    in_desiredPosAcceleration_port.doc("Input port for reading the desired acceleration from Trajectory Generator");
    ports()->addPort(in_desiredPosAcceleration_port);
    in_desiredPosAcceleration_flow = RTT::NoData;
    //4
    in_currentPos_var = Eigen::VectorXf(3);
    in_currentPos_port.setName("in_currentPos_port");
    in_currentPos_port.doc("Input port for reading the current position");
    ports()->addPort(in_currentPos_port);
    in_currentPos_flow = RTT::NoData;
    //5
    in_currentPosVelocity_var = Eigen::VectorXf(3);
    in_currentPosVelocity_port.setName("in_currentPosVelocity_port");
    in_currentPosVelocity_port.doc("Input port for reading the current position velocity");
    ports()->addPort(in_currentPosVelocity_port);
    in_currentPosVelocity_flow = RTT::NoData;
    //6
    in_currentJntVelocity_var = rstrt::kinematics::JointVelocities(DOFsize);
    in_currentJntVelocity_port.setName("in_currentJntVelocity_port");
    in_currentJntVelocity_port.doc("Input port for reading the joint velocity");
    ports()->addPort(in_currentJntVelocity_port);
    in_currentJntVelocity_flow = RTT::NoData;
    //7
    in_desiredOrt_var = Eigen::VectorXf(3);
    in_desiredOrt_port.setName("in_desiredOrt_port");
    in_desiredOrt_port.doc("Input port for reading the desired orientation from Trajectory Generator");
    ports()->addPort(in_desiredOrt_port);
    in_desiredOrt_flow = RTT::NoData;
    //8
    in_desiredOrtVelocity_var = Eigen::VectorXf(3);
    in_desiredOrtVelocity_port.setName("in_desiredOrtVelocity_port");
    in_desiredOrtVelocity_port.doc("Input port for reading the desired orientation velocity from Trajectory Generator");
    ports()->addPort(in_desiredOrtVelocity_port);
    in_desiredOrtVelocity_flow = RTT::NoData;
    //9
    in_desiredOrtAcceleration_var = Eigen::VectorXf(3);
    in_desiredOrtAcceleration_port.setName("in_desiredOrtAcceleration_port");
    in_desiredOrtAcceleration_port.doc("Input port for reading the desired orientation acceleration from Trajectory Generator");
    ports()->addPort(in_desiredOrtAcceleration_port);
    in_desiredOrtAcceleration_flow = RTT::NoData;
    //10
    in_currentOrt_var = Eigen::VectorXf(3);
    in_currentOrt_port.setName("in_currentOrt_port");
    in_currentOrt_port.doc("Input port for reading the current orientation");
    ports()->addPort(in_currentOrt_port);
    in_currentOrt_flow = RTT::NoData;
    //11
    in_currentOrtVelocity_var = Eigen::VectorXf(3);
    in_currentOrtVelocity_port.setName("in_currentOrtVelocity_port");
    in_currentOrtVelocity_port.doc("Input port for reading the current orientation velocity");
    ports()->addPort(in_currentOrtVelocity_port);
    in_currentOrtVelocity_flow = RTT::NoData;
    //12
    in_jacobian_var = Eigen::MatrixXf(6,DOFsize);
    in_jacobian_port.setName("in_jacobian_port");
    in_jacobian_port.doc("Input port for reading the jacobian");
    ports()->addPort(in_jacobian_port);
    in_jacobian_flow = RTT::NoData;
    //13
    in_jacobianDot_var = Eigen::MatrixXf(6,DOFsize);
    in_jacobianDot_port.setName("in_jacobianDot_port");
    in_jacobianDot_port.doc("Input port for reading the derivative of jacobian");
    ports()->addPort(in_jacobianDot_port);
    in_jacobianDot_flow = RTT::NoData;
    //14
    in_constraintLambda_var = Eigen::MatrixXf(6,6);
    in_constraintLambda_port.setName("in_constraintLambda_port");
    in_constraintLambda_port.doc("Input port for reading the constraint Lambda (after Eq. (11) of the paper)");
    ports()->addPort(in_constraintLambda_port);
    in_constraintLambda_flow = RTT::NoData;
    //15
    in_h_var = Eigen::VectorXf(DOFsize);
    in_h_port.setName("in_h_port");
    in_h_port.doc("Input port for reading the constraint Lambda (under Eq. (11) of the paper)");
    ports()->addPort(in_h_port);
    in_h_flow = RTT::NoData;
    //16
    in_constraintM_var = Eigen::MatrixXf(DOFsize,DOFsize);
    in_constraintM_port.setName("in_constraintM_port");
    in_constraintM_port.doc("Input port for reading the constraint inertia matrix (under Eq. (8) of the paper)");
    ports()->addPort(in_constraintM_port);
    in_constraintM_flow = RTT::NoData;
    //17
    in_P_var = Eigen::MatrixXf(DOFsize,DOFsize);
    in_P_port.setName("in_P_port");
    in_P_port.doc("Input port for reading the projection matrix");
    ports()->addPort(in_P_port);
    in_P_flow = RTT::NoData;
    //18
    in_constraintC_var = Eigen::MatrixXf(DOFsize,DOFsize);
    in_constraintC_port.setName("in_constraintC_port");
    in_constraintC_port.doc("Input port for reading the constraint C matrix");
    ports()->addPort(in_constraintC_port);
    in_constraintC_flow = RTT::NoData;

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
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(PositionController)

