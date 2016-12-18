/* Author: Niels Dehio
 * Date:   07 December 2016
 *
 * Description:
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/Timer.hpp>
#include <string>

//#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
//#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/geometry/Rotation.hpp>
#include <rst-rt/geometry/Translation.hpp>
#include <kdl/frames.hpp>


class TrajectoryGeneratorBoris: public RTT::TaskContext {
public:
    TrajectoryGeneratorBoris(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    // call before preparePorts()
	void setTranslationOnly(const bool translationOnly);
    void setWaitTime(double wTime);
    void preparePorts();
    double getSimulationTime();

    void getPosition(double time, Eigen::VectorXf & ret);
    void getVelocity(double time, Eigen::VectorXf & ret);
    void getAcceleration(double time, Eigen::VectorXf & ret);
    void setObjectCenter(double x, double y,double z);
    void setObjectSize(double size);
    void displayStatus();

private:
    // Declare input ports and their datatypes

    // Declare output ports and their datatypes
    RTT::OutputPort<Eigen::VectorXf> out_desiredTaskSpacePosition_port;
    RTT::OutputPort<Eigen::VectorXf> out_desiredTaskSpaceVelocity_port;
    RTT::OutputPort<Eigen::VectorXf> out_desiredTaskSpaceAcceleration_port;

    // Data flow:
    RTT::FlowStatus out_desiredTaskSpacePosition_flow;
    RTT::FlowStatus out_desiredTaskSpaceVelocity_flow;
    RTT::FlowStatus out_desiredTaskSpaceAcceleration_flow;

    // variables
    Eigen::VectorXf out_desiredTaskSpacePosition_var;
    Eigen::VectorXf out_desiredTaskSpaceVelocity_var;
    Eigen::VectorXf out_desiredTaskSpaceAcceleration_var;
    bool portsArePrepared;
    bool sendTranslationOnly;
    unsigned int TaskSpaceDimension;
    RTT::os::TimeService::ticks start_ticks;
    double start_time, wait_time, current_time, time_diff;
    double timescale;
    Eigen::Vector3f TipOrientationEulerZYXAngleLeft, TipOrientationEulerZYXAngleRight, TipOrientationAxisAngleLeft, TipOrientationAxisAngleRight;
    Eigen::Vector3f objectcenter;
    float factor, objectsize;
};

