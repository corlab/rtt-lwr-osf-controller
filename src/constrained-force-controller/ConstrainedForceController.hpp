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

class ConstrainedForceController: public RTT::TaskContext {
public:
	ConstrainedForceController(std::string const & name);

	// RTT::TaskContext methods that are needed for any standard component and
	// should be implemented by user
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	void setDOFsize(unsigned int DOFsize);
    void setLambda(Eigen::VectorXf new_lambda);
	void preparePorts();
    void displayStatus();

private:
	// Declare input ports and their datatypes
	RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;
	RTT::InputPort<Eigen::VectorXf> in_lambda_des_port;
	RTT::InputPort<Eigen::MatrixXf> in_jacobian_c_port;
	RTT::InputPort<Eigen::MatrixXf> in_inertia_port;
	RTT::InputPort<Eigen::MatrixXf> in_inertia_c_port;
	RTT::InputPort<Eigen::MatrixXf> in_p_port;
	RTT::InputPort<Eigen::VectorXf> in_h_port;
	RTT::InputPort<Eigen::MatrixXf> in_Cc_port;

	// Declare output ports and their datatypes
	RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;

	// Data flow:
	RTT::FlowStatus in_robotstatus_flow;
	RTT::FlowStatus in_lambda_des_flow;
	RTT::FlowStatus in_jacobian_c_flow;
	RTT::FlowStatus in_inertia_flow;
	RTT::FlowStatus in_inertia_c_flow;
	RTT::FlowStatus in_p_flow;
	RTT::FlowStatus in_h_flow;
	RTT::FlowStatus in_Cc_flow;

	// variables
	rstrt::robot::JointState in_robotstatus_var;
	Eigen::VectorXf in_lambda_des_var;
	Eigen::MatrixXf in_jacobian_c_var;
	Eigen::MatrixXf in_inertia_var;
	Eigen::MatrixXf in_inertia_c_var;
	Eigen::MatrixXf in_p_var;
	Eigen::VectorXf in_h_var;
	Eigen::MatrixXf in_Cc_var;
    Eigen::VectorXf current_lambda;

	rstrt::dynamics::JointTorques out_torques_var;

	unsigned int DOFsize;
	bool receiveTranslationOnly;
	unsigned int TaskSpaceDimension;
	bool portsArePrepared;
};

