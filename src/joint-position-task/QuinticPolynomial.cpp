#include "QuinticPolynomial.hpp"

QuinticPolynomial::QuinticPolynomial(unsigned int numJoints, double _start_time, double _end_time, Eigen::VectorXd & _Qi, Eigen::VectorXd & _Qf){
	this->numJoints = numJoints;
    this->start_time = _start_time;
    this->end_time   = _end_time;
    this->deltaT = end_time-start_time;
    this->Qi = _Qi;
    this->Qf = _Qf;
    this->deltaQ = Qf - Qi;
}

void QuinticPolynomial::getQ(double time, Eigen::VectorXd & ret){
    if (time >= end_time)
        time = end_time;

    double tau = (time-start_time)/(deltaT);
    for (int i=0; i<numJoints; ++i){
        ret(i) = Qi(i) + deltaQ(i)*(6*std::pow(tau,5.0)-15*std::pow(tau,4.0)+10*std::pow(tau,3.0));
    }
}

void QuinticPolynomial::getQd(double time, Eigen::VectorXd & ret){
    if (time >= end_time)
        time = end_time;

    double tau = (time-start_time)/(deltaT);
    for (int i=0; i<numJoints; ++i){
        ret(i) = deltaQ(i)*(30*std::pow(tau,4.0)-60*std::pow(tau,3.0)+30*std::pow(tau,2.0));
    }
}

void QuinticPolynomial::getQdd(double time, Eigen::VectorXd & ret){
    if (time >= end_time)
        time = end_time;

    double tau = (time-start_time)/(deltaT);
    for (int i=0; i<numJoints; ++i){
        ret(i) = deltaQ(i)*(120*std::pow(tau,3.0)-180*std::pow(tau,2.0)+60*tau);
    }
}

void QuinticPolynomial::setInitialConf(Eigen::VectorXd & init){
    this->Qi = init;
    this->deltaQ = Qf - Qi;
}
