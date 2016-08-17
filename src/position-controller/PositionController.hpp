/* Author: Milad Malekzadeh
 * Date:   16 August 2016
 *
 * Description:
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

//#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
//#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/geometry/Rotation.hpp>
#include <rst-rt/geometry/Translation.hpp>

class PositionController: public RTT::TaskContext {
public:
    PositionController(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsize(unsigned int DOFsize);
    void preparePorts();

private:
    // Declare input ports and their datatypes
    // position
    RTT::InputPort<Eigen::VectorXf> in_desiredPos_port;
    RTT::InputPort<Eigen::VectorXf> in_desiredPosVelocity_port;
    RTT::InputPort<Eigen::VectorXf> in_desiredPosAcceleration_port;

    RTT::InputPort<Eigen::VectorXf> in_currentPos_port;
    RTT::InputPort<Eigen::VectorXf> in_currentPosVelocity_port;
    RTT::InputPort<rstrt::kinematics::JointVelocities> in_currentJntVelocity_port;

    // orientation
    RTT::InputPort<Eigen::VectorXf> in_desiredOrt_port;
    RTT::InputPort<Eigen::VectorXf> in_desiredOrtVelocity_port;
    RTT::InputPort<Eigen::VectorXf> in_desiredOrtAcceleration_port;

    RTT::InputPort<Eigen::VectorXf> in_currentOrt_port;
    RTT::InputPort<Eigen::VectorXf> in_currentOrtVelocity_port;


    RTT::InputPort<Eigen::MatrixXf> in_jacobian_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianDot_port;
    RTT::InputPort<Eigen::MatrixXf> in_constraintLambda_port;
    RTT::InputPort<Eigen::VectorXf> in_h_port;
    RTT::InputPort<Eigen::MatrixXf> in_constraintM_port;
    RTT::InputPort<Eigen::MatrixXf> in_P_port;
    RTT::InputPort<Eigen::MatrixXf> in_constraintC_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;

    // Data flow:
    RTT::FlowStatus in_desiredPos_flow;
    RTT::FlowStatus in_desiredPosVelocity_flow;
    RTT::FlowStatus in_desiredPosAcceleration_flow;

    RTT::FlowStatus in_currentPos_flow;
    RTT::FlowStatus in_currentPosVelocity_flow;
    RTT::FlowStatus in_currentJntVelocity_flow;

    RTT::FlowStatus in_desiredOrt_flow;
    RTT::FlowStatus in_desiredOrtVelocity_flow;
    RTT::FlowStatus in_desiredOrtAcceleration_flow;

    RTT::FlowStatus in_currentOrt_flow;
    RTT::FlowStatus in_currentOrtVelocity_flow;


    RTT::FlowStatus in_jacobian_flow;
    RTT::FlowStatus in_jacobianDot_flow;
    RTT::FlowStatus in_constraintLambda_flow;
    RTT::FlowStatus in_h_flow;
    RTT::FlowStatus in_constraintM_flow;
    RTT::FlowStatus in_P_flow;
    RTT::FlowStatus in_constraintC_flow;


    // variables
    Eigen::VectorXf in_desiredPos_var;
    Eigen::VectorXf in_desiredPosVelocity_var;
    Eigen::VectorXf in_desiredPosAcceleration_var;

    Eigen::VectorXf in_currentPos_var;
    Eigen::VectorXf in_currentPosVelocity_var;
    rstrt::kinematics::JointVelocities in_currentJntVelocity_var;

    Eigen::VectorXf in_desiredOrt_var;
    Eigen::VectorXf in_desiredOrtVelocity_var;
    Eigen::VectorXf in_desiredOrtAcceleration_var;

    Eigen::VectorXf in_currentOrt_var;
    Eigen::VectorXf in_currentOrtVelocity_var;


    Eigen::MatrixXf in_jacobian_var;
    Eigen::MatrixXf in_jacobianDot_var;
    Eigen::MatrixXf in_constraintLambda_var;
    Eigen::VectorXf in_h_var;
    Eigen::MatrixXf in_constraintM_var;
    Eigen::MatrixXf in_P_var;
    Eigen::MatrixXf in_constraintC_var;

    rstrt::dynamics::JointTorques out_torques_var;

    unsigned int DOFsize;
    float gainP_position, gainD_position, gainP_orientation, gainD_orientation;
    bool portsArePrepared;

    // other variables
    Eigen::VectorXf ref_PosAcceleration;
    Eigen::VectorXf ref_OrtAcceleration;

    Eigen::VectorXf ref_Acceleration, constraintForce;


};

