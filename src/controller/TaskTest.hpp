#ifndef TASK_TEST_HPP
#define TASK_TEST_HPP

#include <eigen3/Eigen/Core>

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel.hpp>

class TaskTest {
public:
	TaskTest();
	~TaskTest();
    double radius;
    TaskTest(double _start_time, double _end_time, Eigen::VectorXd _Pi, Eigen::VectorXd _Pf);
    Eigen::VectorXd getPosition(double time);
    Eigen::VectorXd getVelocity(double time);
    Eigen::VectorXd getAcceleration(double time);
    Eigen::VectorXd getPositionOrientation(double time);
    Eigen::VectorXd getVelocityOrientation(double time);
    Eigen::VectorXd getAccelerationOrientation(double time);
	Eigen::VectorXd getDynamicSystem(KDL::Frame& cartFrame, KDL::FrameVel& cartVelFrame, Eigen::VectorXd& yD);
    double start_time;
    double end_time;
    double deltaT;

    Eigen::VectorXd Pi;
    Eigen::VectorXd Pf;
    Eigen::VectorXd deltaP;
    Eigen::MatrixXd BoardRot;
    Eigen::VectorXd BoardTransl, TipOrientation;


};


#endif
