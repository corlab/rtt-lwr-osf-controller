/* Author: Milad Malekzadeh
 * Date:   16 August 2016
 *
 * Description:
 */

#include "TrajectoryGenerator.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


TrajectoryGenerator::TrajectoryGenerator(std::string const & name) : RTT::TaskContext(name), sendTranslationOnly(true) {
    //prepare operations
    addOperation("preparePorts", &TrajectoryGenerator::preparePorts, this).doc("prepare ports");
    addOperation("setTranslationOnly", &TrajectoryGenerator::setTranslationOnly, this, RTT::ClientThread).doc("set translation only, or use also orientation");
    addOperation("setRadius", &TrajectoryGenerator::setRadius, this, RTT::ClientThread).doc("set radius");
    addOperation("setCenter", &TrajectoryGenerator::setCenter,this,RTT::ClientThread).doc("set center");
    addOperation("setWaitTime", &TrajectoryGenerator::setWaitTime,this,RTT::ClientThread).doc("set wait time");
    addProperty("radius",radius);

    //other stuff
    portsArePrepared = false;

    start_time = 0.0;
    wait_time = 0.0;
    _timescale = 0.6;

    BoardRot = Eigen::MatrixXf::Zero(3,3);
    BoardTransl = Eigen::Vector3f::Zero();
    TipOrientationEulerZYXAngle = Eigen::Vector3f::Zero();
    TipOrientationAxisAngle = Eigen::Vector3f::Zero();
    tmp = Eigen::VectorXf(3);

    setTranslationOnly(true);

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
//    TipOrientationEulerZYXAngle(0) = 0;
//    TipOrientationEulerZYXAngle(1) = -M_PI - boardAngle_rad;
//    TipOrientationEulerZYXAngle(2) = 0;
//    radius = 0.15;

    // board parallel to floor
    double boardAngle_deg = 0;
    double boardAngle_rad = boardAngle_deg / 360 * 2 * M_PI;
    BoardRot.fill(0);
    BoardRot(0,0) = cos(boardAngle_rad);
    BoardRot(0,2) = sin(boardAngle_rad);
    BoardRot(1,1) = 1;
    BoardRot(2,0) = -sin(boardAngle_rad);
    BoardRot(2,2) = cos(boardAngle_rad);

    BoardTransl(0) = -0.55;
    BoardTransl(1) = 0;
    BoardTransl(2) = 0.5;

    TipOrientationEulerZYXAngle(0) = 0;
    TipOrientationEulerZYXAngle(1) = -M_PI - boardAngle_rad;
    TipOrientationEulerZYXAngle(2) = 0;

    //convert EulerZYX to AxisAngle
    KDL::Rotation r;
    KDL::Vector kdl_axisangle;
    r = KDL::Rotation::EulerZYX(TipOrientationEulerZYXAngle(0), TipOrientationEulerZYXAngle(1), TipOrientationEulerZYXAngle(2));
    kdl_axisangle = KDL::Vector();
    kdl_axisangle = r.GetRot();
    TipOrientationAxisAngle(0) = kdl_axisangle.x();
    TipOrientationAxisAngle(1) = kdl_axisangle.y();
    TipOrientationAxisAngle(2) = kdl_axisangle.z();

    radius = 0.15;
}

void TrajectoryGenerator::setTranslationOnly(const bool translationOnly) {
	sendTranslationOnly = translationOnly;
	if(sendTranslationOnly){
		TaskSpaceDimension = 3;
	}
	else{
		TaskSpaceDimension = 6;
	}
}

void TrajectoryGenerator::setRadius(float r) {
    assert(r >= 0);
    radius = r;
}

void TrajectoryGenerator::setWaitTime(double wTime) {//use this function with wTime=infinity for reaching to a constant pose only
    assert(wTime >= 0);
    wait_time = wTime;
}

bool TrajectoryGenerator::configureHook() {
    return true;
}

bool TrajectoryGenerator::startHook() {
    this->start_time = getSimulationTime(); //TODO: correct??
    return true;
}

void TrajectoryGenerator::updateHook() {
    current_time = this->getSimulationTime();
    time_diff = this->current_time - this->start_time - this->wait_time;
    if(time_diff < 0.0){
        time_diff = 0.0;
    }

    if(sendTranslationOnly){
        this->getPositionTranslation(time_diff, out_desiredTaskSpacePosition_var);
        this->getVelocityTranslation(time_diff, out_desiredTaskSpaceVelocity_var);
        this->getAccelerationTranslation(time_diff, out_desiredTaskSpaceAcceleration_var);
    }
    else{
        this->getPosition(time_diff, out_desiredTaskSpacePosition_var);
        this->getVelocity(time_diff, out_desiredTaskSpaceVelocity_var);
        this->getAcceleration(time_diff, out_desiredTaskSpaceAcceleration_var);
    }

    if(this->current_time - this->start_time < this->wait_time){
        out_desiredTaskSpaceVelocity_var.setZero();
        out_desiredTaskSpaceAcceleration_var.setZero();
    }

    out_desiredTaskSpacePosition_port.write(out_desiredTaskSpacePosition_var);
    out_desiredTaskSpaceVelocity_port.write(out_desiredTaskSpaceVelocity_var);
    out_desiredTaskSpaceAcceleration_port.write(out_desiredTaskSpaceAcceleration_var);
}

void TrajectoryGenerator::stopHook() {
}

void TrajectoryGenerator::cleanupHook() {
    portsArePrepared = false;
}

void TrajectoryGenerator::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("out_desiredTaskSpacePosition_port");
        ports()->removePort("out_desiredTaskSpaceVelocity_port");
        ports()->removePort("out_desiredTaskSpaceAcceleration_port");
    }

    out_desiredTaskSpacePosition_var = Eigen::VectorXf::Zero(TaskSpaceDimension);
    out_desiredTaskSpacePosition_port.setName("out_desiredTaskSpacePosition_port");
    out_desiredTaskSpacePosition_port.doc("Output port for sending the desired taskspace position");
    out_desiredTaskSpacePosition_port.setDataSample(out_desiredTaskSpacePosition_var);
    ports()->addPort(out_desiredTaskSpacePosition_port);

    out_desiredTaskSpaceVelocity_var = Eigen::VectorXf::Zero(TaskSpaceDimension);
    out_desiredTaskSpaceVelocity_port.setName("out_desiredTaskSpaceVelocity_port");
    out_desiredTaskSpaceVelocity_port.doc("Output port for sending the desired taskspace velocity");
    out_desiredTaskSpaceVelocity_port.setDataSample(out_desiredTaskSpaceVelocity_var);
    ports()->addPort(out_desiredTaskSpaceVelocity_port);

    out_desiredTaskSpaceAcceleration_var = Eigen::VectorXf::Zero(TaskSpaceDimension);
    out_desiredTaskSpaceAcceleration_port.setName("out_desiredTaskSpaceAcceleration_port");
    out_desiredTaskSpaceAcceleration_port.doc("Output port for sending the desired taskspace acceleration");
    out_desiredTaskSpaceAcceleration_port.setDataSample(out_desiredTaskSpaceAcceleration_var);
    ports()->addPort(out_desiredTaskSpaceAcceleration_port);

    portsArePrepared = true;
}

double TrajectoryGenerator::getSimulationTime() {
    return 1E-9
            * RTT::os::TimeService::ticks2nsecs(
                    RTT::os::TimeService::Instance()->getTicks());
}

void TrajectoryGenerator::getPosition(double time, Eigen::VectorXf & ret) {
    tmp(0) = radius * cos(_timescale * time);
    tmp(1) = radius * sin(_timescale * time);
    tmp(2) = 0.0;
    tmp = BoardRot * tmp + BoardTransl;

    ret(0) = tmp(0);
    ret(1) = tmp(1);
    ret(2) = tmp(2);
    ret(3) = TipOrientationAxisAngle(0);
    ret(4) = TipOrientationAxisAngle(1);
    ret(5) = TipOrientationAxisAngle(2);

//    this->getPositionTranslation(time, ret);
//    this->getPositionOrientation(time, ret+3);
}


void TrajectoryGenerator::getVelocity(double time, Eigen::VectorXf & ret) {
    tmp(0) = radius * (-1)*sin(_timescale * time);
    tmp(1) = radius * cos(_timescale * time);
    tmp(2) = 0.0;
    tmp = BoardRot * tmp;

    ret(0) = tmp(0);
    ret(1) = tmp(1);
    ret(2) = tmp(2);
    ret(3) = 0;
    ret(4) = 0;
    ret(5) = 0;

//    this->getVelocityTranslation(time, ret);
//    this->getVelocityOrientation(time, ret+3);
}



void TrajectoryGenerator::getAcceleration(double time, Eigen::VectorXf & ret) {
    tmp(0) = radius * (-1)*cos(_timescale * time);
    tmp(1) = radius * (-1)*sin(_timescale * time);
    tmp(2) = 0.0;
    tmp = BoardRot * tmp;
//    std::cout<<tmp<<"\n--------------------------------------\n";

    ret(0) = tmp(0);
    ret(1) = tmp(1);
    ret(2) = tmp(2);
    ret(3) = 0;
    ret(4) = 0;
    ret(5) = 0;

//    this->getAccelerationTranslation(time, ret);
//    this->getAccelerationOrientation(time, ret+3);
}

void TrajectoryGenerator::getPositionTranslation(double time, Eigen::VectorXf & ret) {
    ret(0) = radius * cos(_timescale * time);
    ret(1) = radius * sin(_timescale * time);
    ret(2) = 0.0;
    ret = BoardRot * ret + BoardTransl;
}

void TrajectoryGenerator::getVelocityTranslation(double time, Eigen::VectorXf & ret) {
    ret(0) = radius * (-1)*sin(_timescale * time);
    ret(1) = radius * cos(_timescale * time);
    ret(2) = 0.0;
    ret = BoardRot * ret;
}

void TrajectoryGenerator::getAccelerationTranslation(double time, Eigen::VectorXf & ret) {
    ret(0) = radius * (-1)*cos(_timescale * time);
    ret(1) = radius * (-1)*sin(_timescale * time);
    ret(2) = 0.0;
    ret = BoardRot * ret;
}

void TrajectoryGenerator::getPositionOrientation(double time, Eigen::VectorXf & ret) {
    ret(0) = TipOrientationAxisAngle(0);
    ret(1) = TipOrientationAxisAngle(1);
    ret(2) = TipOrientationAxisAngle(2);
}

void TrajectoryGenerator::getVelocityOrientation(double time, Eigen::VectorXf & ret) {
    ret(0) = 0;
    ret(1) = 0;
    ret(2) = 0;
}

void TrajectoryGenerator::getAccelerationOrientation(double time, Eigen::VectorXf & ret) {
    ret(0) = 0;
    ret(1) = 0;
    ret(2) = 0;
}

void TrajectoryGenerator::setCenter(double x,double y,double z){
	BoardTransl(0) = x;
	BoardTransl(1) = y;
	BoardTransl(2) = z;
}


//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(TrajectoryGenerator)

