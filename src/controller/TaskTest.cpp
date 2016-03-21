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
    BoardRot.resize(3,3);
    BoardTransl.resize(3);
    TipOrientation.resize(3);

    // board 45 degrees opposite side
//    double boardAngle_deg = 45;
//    double boardAngle_rad = boardAngle_deg / 360 * 2 * M_PI;
//    BoardRot(0,0) = cos(boardAngle_rad);
//    BoardRot(0,2) = sin(boardAngle_rad);
//    BoardRot(1,1) = 1;
//    BoardRot(2,0) = -sin(boardAngle_rad);
//    BoardRot(2,2) = cos(boardAngle_rad);
//
//    BoardTransl(0) = -0.45;
//	BoardTransl(1) = 0.0;
//	BoardTransl(2) = 0.75;//0.7;
//
//    TipOrientation(0) = 0;
//    TipOrientation(1) = -M_PI - boardAngle_rad;
//    TipOrientation(2) = 0;
//    radius = 0.15;

    // board parallel to floor
    double boardAngle_deg = 0;
	double boardAngle_rad = boardAngle_deg / 360 * 2 * M_PI;
	BoardRot(0,0) = cos(boardAngle_rad);
	BoardRot(0,2) = sin(boardAngle_rad);
	BoardRot(1,1) = 1;
	BoardRot(2,0) = -sin(boardAngle_rad);
	BoardRot(2,2) = cos(boardAngle_rad);

	BoardTransl(0) = -0.55;
	BoardTransl(1) = 0.0;
	BoardTransl(2) = 0.5;

	TipOrientation(0) = 0;
	TipOrientation(1) = -M_PI - boardAngle_rad;
	TipOrientation(2) = 0;
	radius = 0.15;

}


void TaskTest::getPosition(double time, Eigen::VectorXd & ret) {
//    if (time >= end_time)
//        time = end_time;

//    if (time < end_time){
//		double tau = (time-start_time)/(deltaT);
//		for (int i=0; i<6; ++i){
//			ret(i) = Pi(i) + deltaP(i)*(6*std::pow(tau,5.0)-15*std::pow(tau,4.0)+10*std::pow(tau,3.0));
//		}
//    }
//    else{
//    	double tau = (time-end_time)/(deltaT)*(2.0*M_PI);
//    	ret = Pf;
//    	ret(1) += 0.5 * sin(tau);
//    	ret(2) += 0.25* (-cos(tau)+1.0);
//    }

    tmp(0) = radius * cos(time-start_time);
    tmp(1) = radius * sin(time-start_time);
    tmp(2) = 0.0;
    tmp = BoardRot * tmp + BoardTransl;

    ret(0) = tmp(0);
    ret(1) = tmp(1);
    ret(2) = tmp(2);
    ret(3) = TipOrientation(0);
    ret(4) = TipOrientation(1);
    ret(5) = TipOrientation(2);

//    this->getPositionTranslation(time, ret);
//    this->getPositionOrientation(time, ret+3);
}


void TaskTest::getVelocity(double time, Eigen::VectorXd & ret) {
//    if (time >= end_time)
//        time = end_time;

//    if (time < end_time){
//		double tau = (time-start_time)/(deltaT);
//		for (int i=0; i<6; ++i){
//			ret(i) = deltaP(i)*(30*std::pow(tau,4.0)-60*std::pow(tau,3.0)+30*std::pow(tau,2.0));
//		}
//    }
//    else{
//    	double tau = (time-end_time)/(deltaT)*(2.0*M_PI);
//    	ret(1) = 0.5 * cos(tau);
//		ret(2) = 0.25 * sin(tau);
//	}

    tmp(0) = radius * (-1)*sin(time-start_time);
    tmp(1) = radius * cos(time-start_time);
    tmp(2) = 0.0;
    tmp = BoardRot * tmp;

    ret(0) = tmp(0);
    ret(1) = tmp(1);
    ret(2) = tmp(2);
    ret(0) = 0;
    ret(1) = 0;
    ret(2) = 0;

//    this->getVelocityTranslation(time, ret);
//    this->getVelocityOrientation(time, ret+3);
}



void TaskTest::getAcceleration(double time, Eigen::VectorXd & ret) {
//    if (time >= end_time)
//        time = end_time;

//    if (time < end_time){
//		double tau = (time-start_time)/(deltaT);
//		for (int i=0; i<6; ++i){
//			ret(i) = deltaP(i)*(120*std::pow(tau,3.0)-180*std::pow(tau,2.0)+60*tau);
//		}
//    }
//    else{
//    	double tau = (time-end_time)/(deltaT)*(2.0*M_PI);
//    	ret(1) = 0.5 * -sin(tau);
//		ret(2) = 0.25 * cos(tau);
//    }

    tmp(0) = radius * (-1)*cos(time-start_time);
    tmp(1) = radius * (-1)*sin(time-start_time);
	tmp(2) = 0.0;
	tmp = BoardRot * tmp;

    ret(0) = tmp(0);
    ret(1) = tmp(1);
    ret(2) = tmp(2);
    ret(3) = 0;
    ret(4) = 0;
    ret(5) = 0;

//    this->getAccelerationTranslation(time, ret);
//    this->getAccelerationOrientation(time, ret+3);
}

void TaskTest::getPositionTranslation(double time, Eigen::VectorXd & ret) {
	ret(0) = radius * cos(time-start_time);
	ret(1) = radius * sin(time-start_time);
	ret(2) = 0.0;
	ret = BoardRot * ret + BoardTransl;
}

void TaskTest::getVelocityTranslation(double time, Eigen::VectorXd & ret) {
	ret(0) = radius * (-1)*sin(time-start_time);
	ret(1) = radius * cos(time-start_time);
	ret(2) = 0.0;
	ret = BoardRot * ret;
}

void TaskTest::getAccelerationTranslation(double time, Eigen::VectorXd & ret) {
	ret(0) = radius * (-1)*cos(time-start_time);
	ret(1) = radius * (-1)*sin(time-start_time);
	ret(2) = 0.0;
	ret = BoardRot * ret;
}

void TaskTest::getPositionOrientation(double time, Eigen::VectorXd & ret) {
    ret(0) = TipOrientation(0);
    ret(1) = TipOrientation(1);
    ret(2) = TipOrientation(2);
}

void TaskTest::getVelocityOrientation(double time, Eigen::VectorXd & ret) {
    ret(0) = 0;
    ret(1) = 0;
    ret(2) = 0;
}

void TaskTest::getAccelerationOrientation(double time, Eigen::VectorXd & ret) {
    ret(0) = 0;
    ret(1) = 0;
    ret(2) = 0;
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
