#pragma once

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/base/RunnableInterface.hpp>
#include <rtt/Activity.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>

#include <iostream>
#include <string>

#include "QuinticPolynomial.hpp"

class RttQuinticPolynomialWrapper: public RTT::TaskContext {
public:
	RttQuinticPolynomialWrapper(std::string const& name, double start_time, unsigned int numJoints);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

protected:
	double getSimulationTime();
	double start_time;
	unsigned int numJoints;

	QuinticPolynomial jnt_task;

	//output ports
	RTT::OutputPort<Eigen::VectorXd> cmdJntPos_Port;
	RTT::OutputPort<Eigen::VectorXd> cmdJntVel_Port;
	RTT::OutputPort<Eigen::VectorXd> cmdJntAcc_Port;


	Eigen::VectorXd cmdJntPos;
	Eigen::VectorXd cmdJntVel;
	Eigen::VectorXd cmdJntAcc;
};

