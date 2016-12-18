/* Author: Niels Dehio
 * Date:   30 August 2016
 *
 * Description: 
 */

#include "ReachingTask.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


ReachingTask::ReachingTask(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setTaskSpaceDimension", &ReachingTask::setTaskSpaceDimension,this, RTT::ClientThread).doc("set TaskSpaceDimension");
    addOperation("setTranslationOnly", &ReachingTask::setTranslationOnly,this, RTT::ClientThread).doc("set translation only, or use also orientation");
    addOperation("setReaching", &ReachingTask::setReaching, this, RTT::ClientThread).doc("set reaching");
    addOperation("setReachingPosition", &ReachingTask::setReachingPosition, this, RTT::ClientThread).doc("set reaching position");
    addOperation("setReachingOrientationAxisAngle", &ReachingTask::setReachingOrientationAxisAngle, this, RTT::ClientThread).doc("set reaching orientation as axis angle");
    addOperation("setReachingOrientationEulerZYXAngle", &ReachingTask::setReachingOrientationEulerZYXAngle, this, RTT::ClientThread).doc("set reaching orientation as euler xyz angle");
    addOperation("preparePorts", &ReachingTask::preparePorts, this, RTT::ClientThread).doc("preparePorts");
    addOperation("printCurrentState", &ReachingTask::printCurrentState, this, RTT::ClientThread).doc("print current state");


    //other stuff
    portsArePrepared = false;
    this->setTranslationOnly(true);
}

bool ReachingTask::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    return true;
}

bool ReachingTask::startHook() {
    // this method starts the component
    if (!out_reachingposition_port.connected() || !out_reachingvelocity_port.connected() || !out_reachingacceleration_port.connected())
        return false;
    else
        return true;
}

void ReachingTask::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    out_reachingposition_port.write(out_reachingposition_var);
    out_reachingvelocity_port.write(out_reachingvelocity_var);
    out_reachingacceleration_port.write(out_reachingacceleration_var);
}

void ReachingTask::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void ReachingTask::cleanupHook() {
    // cleaning the component data
    portsArePrepared = false;
}

void ReachingTask::setTaskSpaceDimension(const unsigned int TaskSpaceDimension) {
    this->TaskSpaceDimension = TaskSpaceDimension;
}

void ReachingTask::setTranslationOnly(const bool translationOnly) {
}

void ReachingTask::setReaching(Eigen::VectorXf const & reachingtarget){
    assert(reachingtarget.size() == TaskSpaceDimension);
    out_reachingposition_var = reachingtarget;
}

void ReachingTask::setReachingPosition(float const x, float const y, float const z){
    assert(TaskSpaceDimension==3 || TaskSpaceDimension==6);
    out_reachingposition_var(0) = x;
    out_reachingposition_var(1) = y;
    out_reachingposition_var(2) = z;
}

void ReachingTask::setReachingOrientationAxisAngle(float const x, float const y, float const z){
    assert(TaskSpaceDimension==6);
    out_reachingposition_var(3) = x;
    out_reachingposition_var(4) = y;
    out_reachingposition_var(5) = z;
}

void ReachingTask::setReachingOrientationEulerZYXAngle(float const alpha, float const beta, float const gamma){
    assert(TaskSpaceDimension==6);
    KDL::Rotation r;
    KDL::Vector kdl_axisangle;
    r = KDL::Rotation::EulerZYX(alpha, beta, gamma);
    kdl_axisangle = KDL::Vector();
    kdl_axisangle = r.GetRot();
    out_reachingposition_var(3) = kdl_axisangle.x();
    out_reachingposition_var(4) = kdl_axisangle.y();
    out_reachingposition_var(5) = kdl_axisangle.z();
}

void ReachingTask::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("out_reachingposition_port");
        ports()->removePort("out_reachingvelocity_port");
        ports()->removePort("out_reachingacceleration_port");
    }

    //prepare input

    //prepare output
    out_reachingposition_var = Eigen::VectorXf::Zero(TaskSpaceDimension);
    out_reachingposition_port.setName("out_reachingposition_port");
    out_reachingposition_port.doc("Output port for sending reachingposition vector");
    out_reachingposition_port.setDataSample(out_reachingposition_var);
    ports()->addPort(out_reachingposition_port);

    out_reachingvelocity_var = Eigen::VectorXf::Zero(TaskSpaceDimension);
    out_reachingvelocity_port.setName("out_reachingvelocity_port");
    out_reachingvelocity_port.doc("Output port for sending reachingvelocity vector");
    out_reachingvelocity_port.setDataSample(out_reachingvelocity_var);
    ports()->addPort(out_reachingvelocity_port);

    out_reachingacceleration_var = Eigen::VectorXf::Zero(TaskSpaceDimension);
    out_reachingacceleration_port.setName("out_reachingacceleration_port");
    out_reachingacceleration_port.doc("Output port for sending reachingacceleration vector");
    out_reachingacceleration_port.setDataSample(out_reachingacceleration_var);
    ports()->addPort(out_reachingacceleration_port);

    portsArePrepared = true;
}

void ReachingTask::printCurrentState(){
    std::cout << "############## CartesianTrackingTask State begin " << std::endl;
    std::cout << " reachingpoint " << out_reachingposition_var << std::endl;
    std::cout << " reachingvelocity " << out_reachingvelocity_var << std::endl;
    std::cout << " reachingacceleration " << out_reachingacceleration_var << std::endl;
    std::cout << "############## CartesianTrackingTask State end " << std::endl;
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(ReachingTask)
