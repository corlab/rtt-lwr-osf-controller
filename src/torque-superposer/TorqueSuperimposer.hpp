/* Author: Niels Dehio
 * Date:   16 August 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/robot/Weights.hpp>
#include <boost/lexical_cast.hpp>

class TorqueSuperimposer: public RTT::TaskContext {
public:
    TorqueSuperimposer(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsize(unsigned int DOFsize);
    void setConstrainedVersionMode(bool useConstrainedVersion);
    bool setInitialWeights(rstrt::robot::Weights & initialWeights);
    bool setWeights(float weightPos, float weightNull, float weightForce);
    void preparePorts();
    bool isnanVector(Eigen::VectorXf const & vec);
    void displayStatus();

private:
    // Declare input ports and their datatypes
    RTT::InputPort<rstrt::robot::Weights> in_weights_port;
    RTT::InputPort<Eigen::MatrixXf> in_projection_port;
    RTT::InputPort<rstrt::dynamics::JointTorques> in_torquesA_port;
    RTT::InputPort<rstrt::dynamics::JointTorques> in_torquesB_port;
    RTT::InputPort<rstrt::dynamics::JointTorques> in_torquesC_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;

    // Data flow:
    RTT::FlowStatus in_weights_flow;
    RTT::FlowStatus in_projection_flow;
    RTT::FlowStatus in_torquesA_flow;
    RTT::FlowStatus in_torquesB_flow;
    RTT::FlowStatus in_torquesC_flow;

    // variables
    rstrt::robot::Weights in_weights_var;
    Eigen::MatrixXf in_projection_var;
    rstrt::dynamics::JointTorques in_torquesA_var;
    rstrt::dynamics::JointTorques in_torquesB_var;
    rstrt::dynamics::JointTorques in_torquesC_var;
    rstrt::dynamics::JointTorques out_torques_var;
    unsigned int DOFsize;
    Eigen::MatrixXf identityDOFsizeDOFsize;
    rstrt::robot::Weights current_weights;
    bool portsArePrepared;
    bool useConstrainedVersion;
};

