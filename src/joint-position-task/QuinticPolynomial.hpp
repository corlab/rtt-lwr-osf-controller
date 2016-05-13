#pragma once

#include <eigen3/Eigen/Core>
#include <cmath>

class QuinticPolynomial
{
public:
    QuinticPolynomial(unsigned int numJoints, double _start_time, double _end_time, Eigen::VectorXd & _Qi, Eigen::VectorXd & _Qf);
    void getQ(double time, Eigen::VectorXd & ret);
    void getQd(double time, Eigen::VectorXd & ret);
    void getQdd(double time, Eigen::VectorXd & ret);
    void setInitialConf(Eigen::VectorXd & init);

protected:
	unsigned int numJoints;
    double start_time;
    double end_time;
    double deltaT;

    Eigen::VectorXd Qi;
    Eigen::VectorXd Qf;
    Eigen::VectorXd deltaQ;
};

