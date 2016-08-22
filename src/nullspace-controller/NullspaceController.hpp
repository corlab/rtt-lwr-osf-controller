/* Author: Niels Dehio
 * Date:   16 August 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>

class NullspaceController: public RTT::TaskContext {
public:
    NullspaceController(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsize(unsigned int DOFsize);
    void setGains(float kp, float kd);
    bool setDesiredAngles(rstrt::kinematics::JointAngles desiredAngles);
    void preparePorts();
    void displayStatus();

private:
    // Declare input ports and their datatypes
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;
    RTT::InputPort<rstrt::kinematics::JointAngles> in_desiredAngles_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobian_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianInv_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;

    // Data flow:
    RTT::FlowStatus in_robotstatus_flow;
    RTT::FlowStatus in_desiredAngles_flow;
    RTT::FlowStatus in_jacobian_flow;
    RTT::FlowStatus in_jacobianInv_flow;

    // variables
    rstrt::robot::JointState in_robotstatus_var;
    rstrt::kinematics::JointAngles in_desiredAngles_var;
    Eigen::MatrixXf in_jacobian_var;
    Eigen::MatrixXf in_jacobianInv_var;
    rstrt::dynamics::JointTorques out_torques_var;
    Eigen::MatrixXf identityDOFsizeDOFsize;
    rstrt::kinematics::JointAngles current_desiredAngles;
    unsigned int DOFsize;
    bool receiveTranslationOnly;
    unsigned int TaskSpaceDimension;
    float gainP, gainD;
    bool portsArePrepared;
};

