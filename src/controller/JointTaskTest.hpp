#ifndef JOINTTASKTEST_HPP
#define JOINTTASKTEST_HPP


#include <Eigen/Core>

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel.hpp>

class JointTaskTest
{
public:
    JointTaskTest();
    Eigen::VectorXd getJointPosition(double time);
    Eigen::VectorXd getJointVelocity(double time);
    Eigen::VectorXd getJointAcceleration(double time);
};

#endif // JOINTTASKTEST_HPP
