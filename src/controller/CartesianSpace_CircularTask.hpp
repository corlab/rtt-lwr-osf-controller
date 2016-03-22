#pragma once

#include <eigen3/Eigen/Core>

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel.hpp>

class CartesianSpace_CircularTask {
public:
	CartesianSpace_CircularTask(){ };
	CartesianSpace_CircularTask(double _start_time, double _timescale);
	~CartesianSpace_CircularTask();

    void getPosition(double time, Eigen::VectorXd & ret);
    void getVelocity(double time, Eigen::VectorXd & ret);
    void getAcceleration(double time, Eigen::VectorXd & ret);

    void getPositionTranslation(double time, Eigen::VectorXd & ret);
	void getVelocityTranslation(double time, Eigen::VectorXd & ret);
	void getAccelerationTranslation(double time, Eigen::VectorXd & ret);

	void getPositionOrientation(double time, Eigen::VectorXd & ret);
    void getVelocityOrientation(double time, Eigen::VectorXd & ret);
    void getAccelerationOrientation(double time, Eigen::VectorXd & ret);

    double radius;
    double start_time;
    double _timescale;

    Eigen::MatrixXd BoardRot;
    Eigen::VectorXd BoardTransl, TipOrientation;
    Eigen::VectorXd tmp;


};

