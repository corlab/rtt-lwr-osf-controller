/* Author: Milad Malekzadeh
 * Date:   16 August 2016
 *
 * Description:
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>

#include <Eigen/QR>

class ForceEstimator: public RTT::TaskContext {
public:
	ForceEstimator(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsize(unsigned int DOFsize);
    void setFloatingBaseMode(bool fbMode);
    void setTaskSpaceDimension(const unsigned int TaskSpaceDimension);
    void setCstrSpaceDimension(const unsigned int CstrSpaceDimension);
    void preparePorts();
    void checkQR(Eigen::MatrixXf const & A, Eigen::MatrixXf const & Q, Eigen::MatrixXf const & R);
    float getSumOfMatrix(Eigen::MatrixXf const & mat);
    void displayStatus();

private:
    // Declare input ports and their datatypes
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianCstr_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianDotCstr_port;
    RTT::InputPort<Eigen::MatrixXf> in_inertia_port;
    RTT::InputPort<Eigen::VectorXf> in_h_port;
    RTT::InputPort<rstrt::dynamics::JointTorques> in_torques_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<Eigen::VectorXf> out_forceA_port;
    RTT::OutputPort<Eigen::VectorXf> out_forceB_port;

    // Data flow:
    RTT::FlowStatus in_robotstatus_flow;
    RTT::FlowStatus in_jacobianCstr_flow;
    RTT::FlowStatus in_jacobianDotCstr_flow;
    RTT::FlowStatus in_inertia_flow;
    RTT::FlowStatus in_h_flow;
    RTT::FlowStatus in_torques_flow;

    // variables
    rstrt::robot::JointState in_robotstatus_var;
    Eigen::MatrixXf in_jacobianCstr_var;
    Eigen::MatrixXf in_jacobianDotCstr_var;
    Eigen::MatrixXf in_inertia_var;
    Eigen::VectorXf in_h_var;
    rstrt::dynamics::JointTorques in_torques_var;
    Eigen::VectorXf out_forceA_var;
    Eigen::VectorXf out_forceB_var;
    Eigen::MatrixXf identityDOFsizeDOFsize, identityCSdimCSdim, identityRankRank;
    Eigen::MatrixXf tmpeyeCSdimCSdim;
    Eigen::HouseholderQR<Eigen::MatrixXf> qrDecomposition;
    Eigen::MatrixXf B, Sc, Q, R, Rsmall;
    unsigned int DOFsize, floatingBaseDOF, rank;
    unsigned int TaskSpaceDimension, CstrSpaceDimension;
    bool portsArePrepared;
};

