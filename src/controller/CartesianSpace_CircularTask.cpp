#include "CartesianSpace_CircularTask.hpp"
#include <rtt/RTT.hpp>
#include <cmath>


CartesianSpace_CircularTask::CartesianSpace_CircularTask(double _start_time){
    this->start_time = _start_time;

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

CartesianSpace_CircularTask::~CartesianSpace_CircularTask() {

}

void CartesianSpace_CircularTask::getPosition(double time, Eigen::VectorXd & ret) {
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


void CartesianSpace_CircularTask::getVelocity(double time, Eigen::VectorXd & ret) {
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



void CartesianSpace_CircularTask::getAcceleration(double time, Eigen::VectorXd & ret) {
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

void CartesianSpace_CircularTask::getPositionTranslation(double time, Eigen::VectorXd & ret) {
	ret(0) = radius * cos(time-start_time);
	ret(1) = radius * sin(time-start_time);
	ret(2) = 0.0;
	ret = BoardRot * ret + BoardTransl;
}

void CartesianSpace_CircularTask::getVelocityTranslation(double time, Eigen::VectorXd & ret) {
	ret(0) = radius * (-1)*sin(time-start_time);
	ret(1) = radius * cos(time-start_time);
	ret(2) = 0.0;
	ret = BoardRot * ret;
}

void CartesianSpace_CircularTask::getAccelerationTranslation(double time, Eigen::VectorXd & ret) {
	ret(0) = radius * (-1)*cos(time-start_time);
	ret(1) = radius * (-1)*sin(time-start_time);
	ret(2) = 0.0;
	ret = BoardRot * ret;
}

void CartesianSpace_CircularTask::getPositionOrientation(double time, Eigen::VectorXd & ret) {
    ret(0) = TipOrientation(0);
    ret(1) = TipOrientation(1);
    ret(2) = TipOrientation(2);
}

void CartesianSpace_CircularTask::getVelocityOrientation(double time, Eigen::VectorXd & ret) {
    ret(0) = 0;
    ret(1) = 0;
    ret(2) = 0;
}

void CartesianSpace_CircularTask::getAccelerationOrientation(double time, Eigen::VectorXd & ret) {
    ret(0) = 0;
    ret(1) = 0;
    ret(2) = 0;
}

