/* Author: Niels Dehio
 * Date:   30 August 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <Eigen/Dense>
#include <kdl/frames.hpp>


class ReachingTask: public RTT::TaskContext {
public:
    ReachingTask(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setTranslationOnly(const bool translationOnly); // call before preparePorts()
    void setReaching(Eigen::VectorXf const & reachingtarget);
    void setReachingPosition(float const x, float const y, float const z);
    void setReachingOrientationAxisAngle(float const x, float const y, float const z);
    void setReachingOrientationEulerZYXAngle(float const alpha, float const beta, float const gamma);
    void preparePorts();
    void printCurrentState();

private:
    // Declare input ports and their datatypes


    // Declare output ports and their datatypes
    RTT::OutputPort<Eigen::VectorXf> out_reachingposition_port;
    RTT::OutputPort<Eigen::VectorXf> out_reachingvelocity_port;
    RTT::OutputPort<Eigen::VectorXf> out_reachingacceleration_port;

    // Data flow:

    // variables
    Eigen::VectorXf out_reachingposition_var;
    Eigen::VectorXf out_reachingvelocity_var;
    Eigen::VectorXf out_reachingacceleration_var;
    unsigned int TaskSpaceDimension;
    bool portsArePrepared;
};

