#ifndef QUINTICPOLYNOMIAL_HPP
#define QUINTICPOLYNOMIAL_HPP


#include <eigen3/Eigen/Core>

class QuinticPolynomial
{
public:
    double start_time;
    double end_time;
    double deltaT;

    Eigen::VectorXd Qi;
    Eigen::VectorXd Qf;
    Eigen::VectorXd deltaQ;

    QuinticPolynomial();
    QuinticPolynomial(double _start_time, double _end_time, Eigen::VectorXd _Qi, Eigen::VectorXd _Qf);
    Eigen::VectorXd getQ(double time);
    Eigen::VectorXd getQd(double time);
    Eigen::VectorXd getQdd(double time);

    void setInitialConf(Eigen::VectorXd init);


};

#endif // QUINTICPOLYNOMIAL_HPP
