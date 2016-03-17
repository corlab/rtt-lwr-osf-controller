#include "JointTaskTest.hpp"

JointTaskTest::JointTaskTest()
{
}

Eigen::VectorXd JointTaskTest::getJointPosition(double time){
    Eigen::VectorXd ret(7);
    ret(0) = 0.1;
    ret(1) = 0.2;
    ret(2) = 0.3;
    ret(3) = 0.4;
    ret(4) = 0.5;
    ret(5) = 0.6;
    ret(6) = 0.7;
    return ret;
}

Eigen::VectorXd JointTaskTest::getJointVelocity(double time){
    Eigen::VectorXd ret(7);
    ret(0) = time*0.2;
    ret(1) = time*0.2;
    ret(2) = time*0.2;
    ret(3) = 0;
    ret(4) = 0;
    ret(5) = 0;
    ret(6) = 0;
    return ret;
}

Eigen::VectorXd JointTaskTest::getJointAcceleration(double time){
    Eigen::VectorXd ret(7);
    ret(0) = 0.2;
    ret(1) = 0.2;
    ret(2) = 0.2;
    ret(3) = 0;
    ret(4) = 0;
    ret(5) = 0;
    ret(6) = 0;
    return ret;
}
