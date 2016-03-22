#include "TaskTest.hpp"
#include <rtt/RTT.hpp>
#include <cmath>


TaskTest::TaskTest() : radius(0.3) {

}

TaskTest::~TaskTest() {

}
TaskTest::TaskTest(double _start_time, double _end_time, Eigen::VectorXd _Pi, Eigen::VectorXd _Pf){
    this->start_time = _start_time;
    this->end_time   = _end_time;
    this->deltaT = end_time-start_time;
    this->Pi = _Pi;
    this->Pf = _Pf;
    this->deltaP = Pf - Pi;

    tmp.resize(3);
}


void TaskTest::getPosition(double time, Eigen::VectorXd & ret) {
    if (time >= end_time)
        time = end_time;

//    if (time < end_time){
		double tau = (time-start_time)/(deltaT);
		for (int i=0; i<6; ++i){
			ret(i) = Pi(i) + deltaP(i)*(6*std::pow(tau,5.0)-15*std::pow(tau,4.0)+10*std::pow(tau,3.0));
		}
//    }
//    else{
//
//    }
}


void TaskTest::getVelocity(double time, Eigen::VectorXd & ret) {
    if (time >= end_time)
        time = end_time;

//    if (time < end_time){
		double tau = (time-start_time)/(deltaT);
		for (int i=0; i<6; ++i){
			ret(i) = deltaP(i)*(30*std::pow(tau,4.0)-60*std::pow(tau,3.0)+30*std::pow(tau,2.0));
		}
//    }
//    else{
//
//	}
}



void TaskTest::getAcceleration(double time, Eigen::VectorXd & ret) {
    if (time >= end_time)
        time = end_time;

//    if (time < end_time){
		double tau = (time-start_time)/(deltaT);
		for (int i=0; i<6; ++i){
			ret(i) = deltaP(i)*(120*std::pow(tau,3.0)-180*std::pow(tau,2.0)+60*tau);
		}
//    }
//    else{
//
//    }
}

Eigen::VectorXd TaskTest::getDynamicSystem(KDL::Frame& cartFrame, KDL::FrameVel& cartVelFrame, Eigen::VectorXd& yD) {
	//TODO: don't allocate memmory here!

	Eigen::MatrixXd kp(6, 6);
	kp = 0.1 * Eigen::MatrixXd::Identity(6, 6);

	Eigen::MatrixXd kv(6, 6);
	kv = 2 * std::sqrt(0.1) * Eigen::MatrixXd::Identity(6, 6);


	//RTT::log(RTT::Error) << "yD: " << yD << RTT::endlog();

	//conversion
	Eigen::VectorXd cart(6);
	cart(0) = cartFrame.p.x();
	cart(1) = cartFrame.p.y();
	cart(2) = cartFrame.p.z();
	cart(3) = 0;
	cart(4) = 0;
	cart(5) = 0;
//	cart(3) = cartFrame.M.GetRot().x();
//	cart(4) = cartFrame.M.GetRot().y();
//	cart(5) = cartFrame.M.GetRot().z();

	//RTT::log(RTT::Error) << "cart:\n" << cart << "\n\ncartFrame\n" << cartFrame << RTT::endlog();

	Eigen::VectorXd vel(6);
	vel(0) = cartVelFrame.p.v.x();
	vel(1) = cartVelFrame.p.v.y();
	vel(2) = cartVelFrame.p.v.z();
	vel(3) = 0;
	vel(4) = 0;
	vel(5) = 0;
//	vel(3) = cartVelFrame.GetFrame().M.GetRot().x();
//	vel(4) = cartVelFrame.GetFrame().M.GetRot().y();
//	vel(5) = cartVelFrame.GetFrame().M.GetRot().z();

	//RTT::log(RTT::Error) << "vel:\n" << vel << "\n\ncartVelFrame\n" << cartVelFrame.GetFrame() << RTT::endlog();

	Eigen::VectorXd retAcc(6);
	retAcc = kp * (yD - cart) - kv * vel;

	return retAcc;
}
