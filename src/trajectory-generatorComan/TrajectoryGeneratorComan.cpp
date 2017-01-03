/* Author: Niels Dehio
 * Date:   07 December 2016
 *
 * Description:
 */

#include "TrajectoryGeneratorComan.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


TrajectoryGeneratorComan::TrajectoryGeneratorComan(std::string const & name) : RTT::TaskContext(name), sendTranslationOnly(true) {
    //prepare operations
    addOperation("preparePorts", &TrajectoryGeneratorComan::preparePorts, this).doc("prepare ports");
    addOperation("setTranslationOnly", &TrajectoryGeneratorComan::setTranslationOnly, this, RTT::ClientThread).doc("set translation only, or use also orientation");
    addOperation("setWaitTime", &TrajectoryGeneratorComan::setWaitTime,this,RTT::ClientThread).doc("set wait time");
    addOperation("setObjectCenterLeft", &TrajectoryGeneratorComan::setObjectCenterLeft,this,RTT::ClientThread).doc("set object center left");
    addOperation("setObjectCenterRight", &TrajectoryGeneratorComan::setObjectCenterRight,this,RTT::ClientThread).doc("set object center right");
    addOperation("displayStatus", &TrajectoryGeneratorComan::displayStatus,this,RTT::ClientThread).doc("display status");
    addProperty("factor",factor);

    //other stuff
    portsArePrepared = false;

    start_time = 0.0;
    timescale = 0.6;

    objectcenterLeft = Eigen::Vector3f::Zero();
    objectcenterRight = Eigen::Vector3f::Zero();
    factor = 0;
    TipOrientationEulerZYXAngleLeft = Eigen::Vector3f::Zero();
    TipOrientationEulerZYXAngleRight = Eigen::Vector3f::Zero();
    TipOrientationAxisAngleLeft = Eigen::Vector3f::Zero();
    TipOrientationAxisAngleRight = Eigen::Vector3f::Zero();

    setTranslationOnly(false);

//    TipOrientationEulerZYXAngleLeft(0) = 0;
//    TipOrientationEulerZYXAngleLeft(1) = 0;
//    TipOrientationEulerZYXAngleLeft(2) = 0;
//    TipOrientationEulerZYXAngleRight(0) = 0;
//    TipOrientationEulerZYXAngleRight(1) = 0;
//    TipOrientationEulerZYXAngleRight(2) = 0;

//    //convert EulerZYX to AxisAngle
//    KDL::Rotation r;
//    KDL::Vector kdl_axisangle;
//    r = KDL::Rotation::EulerZYX(TipOrientationEulerZYXAngleLeft(0), TipOrientationEulerZYXAngleLeft(1), TipOrientationEulerZYXAngleLeft(2));
//    kdl_axisangle = KDL::Vector();
//    kdl_axisangle = r.GetRot();
//    TipOrientationAxisAngleLeft(0) = kdl_axisangle.x();
//    TipOrientationAxisAngleLeft(1) = kdl_axisangle.y();
//    TipOrientationAxisAngleLeft(2) = kdl_axisangle.z();

//    r = KDL::Rotation::EulerZYX(TipOrientationEulerZYXAngleRight(0), TipOrientationEulerZYXAngleRight(1), TipOrientationEulerZYXAngleRight(2));
//    kdl_axisangle = KDL::Vector();
//    kdl_axisangle = r.GetRot();
//    TipOrientationAxisAngleRight(0) = kdl_axisangle.x();
//    TipOrientationAxisAngleRight(1) = kdl_axisangle.y();
//    TipOrientationAxisAngleRight(2) = kdl_axisangle.z();

    TipOrientationAxisAngleLeft(0) = 0.0;
    TipOrientationAxisAngleLeft(1) =-1.57;
    TipOrientationAxisAngleLeft(2) = 0.0;

    TipOrientationAxisAngleRight(0) = 0.0;
    TipOrientationAxisAngleRight(1) =-1.57;
    TipOrientationAxisAngleRight(2) = 0.0;
}

void TrajectoryGeneratorComan::setTranslationOnly(const bool translationOnly) {
	sendTranslationOnly = translationOnly;
	if(sendTranslationOnly){
        TaskSpaceDimension = 6;
	}
	else{
        TaskSpaceDimension = 12;
	}
}

void TrajectoryGeneratorComan::setWaitTime(double wTime) {//use this function with wTime=infinity for reaching to a constant pose only
    assert(wTime >= 0);
    wait_time = wTime;
}

bool TrajectoryGeneratorComan::configureHook() {
    return true;
}

bool TrajectoryGeneratorComan::startHook() {
    this->start_time = getSimulationTime(); //TODO: correct??
    return true;
}

void TrajectoryGeneratorComan::updateHook() {
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

void TrajectoryGeneratorComan::stopHook() {
}

void TrajectoryGeneratorComan::cleanupHook() {
    portsArePrepared = false;
}

void TrajectoryGeneratorComan::preparePorts(){
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

double TrajectoryGeneratorComan::getSimulationTime() {
    return 1E-9
            * RTT::os::TimeService::ticks2nsecs(
                    RTT::os::TimeService::Instance()->getTicks());
}

void TrajectoryGeneratorComan::getPositionTranslation(double time, Eigen::VectorXf & ret) {
    double transition = 9995.0;
    if (time < transition){
        ret(0) = objectcenterLeft(0);
        ret(1) = objectcenterLeft(1);
        ret(2) = objectcenterLeft(2);
    }
    else{
        ret(0) = objectcenterLeft(0);
        ret(1) = objectcenterLeft(1);
        ret(2) = objectcenterLeft(2);
    }

    if (time < transition){
        ret(3) = objectcenterRight(0);
        ret(4) = objectcenterRight(1);
        ret(5) = objectcenterRight(2);
    }
    else{
        ret(3) = objectcenterRight(0);
        ret(4) = objectcenterRight(1);
        ret(5) = objectcenterRight(2);
    }
}

void TrajectoryGeneratorComan::getVelocityTranslation(double time, Eigen::VectorXf & ret) {
    ret(0) = 0;
    ret(1) = 0;
    ret(2) = 0;

    ret(3) = 0;
    ret(4) = 0;
    ret(5) = 0;
}

void TrajectoryGeneratorComan::getAccelerationTranslation(double time, Eigen::VectorXf & ret) {
    ret(0) = 0;
    ret(1) = 0;
    ret(2) = 0;

    ret(3) = 0;
    ret(4) = 0;
    ret(5) = 0;
}

void TrajectoryGeneratorComan::getPosition(double time, Eigen::VectorXf & ret) {
    double transition = 9995.0;
    if (time < transition){
        ret(0) = objectcenterLeft(0);
        ret(1) = objectcenterLeft(1);// + 0.3 - time / transition * 0.3;
        ret(2) = objectcenterLeft(2);
    }
    else{
        ret(0) = objectcenterLeft(0);
        ret(1) = objectcenterLeft(1);
        ret(2) = objectcenterLeft(2) + factor * 0.5 * ((-1) * cos(timescale * (time-transition)) +1.0);
    }
    ret(3) = TipOrientationAxisAngleLeft(0);
    ret(4) = TipOrientationAxisAngleLeft(1);
    ret(5) = TipOrientationAxisAngleLeft(2);

    if (time < transition){
        ret(6) = objectcenterRight(0);
        ret(7) = objectcenterRight(1);// - 0.3 + time / transition * 0.3;
        ret(8) = objectcenterRight(2);
    }
    else{
        ret(6) = objectcenterRight(0);
        ret(7) = objectcenterRight(1);
        ret(8) = objectcenterRight(2) + factor * 0.5 * ((-1) * cos(timescale * (time-transition)) +1.0);
    }
    ret(9) = TipOrientationAxisAngleRight(0);
    ret(10)= TipOrientationAxisAngleRight(1);
    ret(11)= TipOrientationAxisAngleRight(2);
}

void TrajectoryGeneratorComan::getVelocity(double time, Eigen::VectorXf & ret) {
    ret(0) = 0;
    ret(1) = 0;
    ret(2) = 0;
    ret(3) = 0;
    ret(4) = 0;
    ret(5) = 0;

    ret(6) = 0;
    ret(7) = 0;
    ret(8) = 0;
    ret(9) = 0;
    ret(10)= 0;
    ret(11)= 0;
}

void TrajectoryGeneratorComan::getAcceleration(double time, Eigen::VectorXf & ret) {
    ret(0) = 0;
    ret(1) = 0;
    ret(2) = 0;
    ret(3) = 0;
    ret(4) = 0;
    ret(5) = 0;

    ret(6) = 0;
    ret(7) = 0;
    ret(8) = 0;
    ret(9) = 0;
    ret(10)= 0;
    ret(11)= 0;
}


void TrajectoryGeneratorComan::setObjectCenterLeft(double x,double y,double z){
    objectcenterLeft(0) = x;
    objectcenterLeft(1) = y;
    objectcenterLeft(2) = z;
}

void TrajectoryGeneratorComan::setObjectCenterRight(double x,double y,double z){
    objectcenterRight(0) = x;
    objectcenterRight(1) = y;
    objectcenterRight(2) = z;
}

void TrajectoryGeneratorComan::displayStatus() {
    RTT::log(RTT::Info) << "out_desiredTaskSpacePosition_var \n" << out_desiredTaskSpacePosition_var << RTT::endlog();
    RTT::log(RTT::Info) << "out_desiredTaskSpaceVelocity_var \n" << out_desiredTaskSpaceVelocity_var << RTT::endlog();
    RTT::log(RTT::Info) << "out_desiredTaskSpaceAcceleration_var \n" << out_desiredTaskSpaceAcceleration_var << RTT::endlog();
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(TrajectoryGeneratorComan)

