/* Author: Milad Malekzadeh
 * Date:   16 August 2016
 *
 * Description:
 */

#include "ForceEstimator.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

ForceEstimator::ForceEstimator(std::string const & name) :
        RTT::TaskContext(name) {
	//prepare operations
	addOperation("setDOFsize", &ForceEstimator::setDOFsize, this).doc("set DOF size");
    addOperation("setFloatingBaseMode", &ForceEstimator::setFloatingBaseMode, this).doc("set FloatingBase mode");
    addOperation("setTaskSpaceDimension", &ForceEstimator::setTaskSpaceDimension,this, RTT::ClientThread).doc("set TaskSpaceDimension");
    addOperation("setCstrSpaceDimension", &ForceEstimator::setCstrSpaceDimension,this, RTT::ClientThread).doc("set CstrSpaceDimension");
	addOperation("displayStatus", &ForceEstimator::displayStatus, this).doc("print status");
	addOperation("preparePorts", &ForceEstimator::preparePorts, this).doc("preparePorts");

    //other stuff
	portsArePrepared = false;
}

bool ForceEstimator::configureHook() {
	return true;
}

bool ForceEstimator::startHook() {
	if (!in_robotstatus_port.connected()) {
		RTT::log(RTT::Info) << "in_robotstatus_port not connected"
				<< RTT::endlog();
		return false;
	}
    if (!in_jacobianCstr_port.connected()) {
        RTT::log(RTT::Info) << "in_jacobianCstr_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_jacobianDotCstr_port.connected()) {
        RTT::log(RTT::Info) << "in_jacobianDotCstr_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_inertia_port.connected()) {
        RTT::log(RTT::Info) << "in_inertia_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_h_port.connected()) {
        RTT::log(RTT::Info) << "in_h_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_torques_port.connected()) {
        RTT::log(RTT::Info) << "in_torques_port not connected"
                << RTT::endlog();
        return false;
    }

    identityRankRank = Eigen::MatrixXf::Identity(rank,rank);
    B = Eigen::MatrixXf::Zero(DOFsize,DOFsize+floatingBaseDOF);
    Sc = Eigen::MatrixXf::Zero(rank,DOFsize+floatingBaseDOF);
    Q = Eigen::MatrixXf::Zero(DOFsize+floatingBaseDOF,DOFsize+floatingBaseDOF);
    R = Eigen::MatrixXf::Zero(DOFsize+floatingBaseDOF, CstrSpaceDimension);
    Rsmall = Eigen::MatrixXf::Zero(CstrSpaceDimension, CstrSpaceDimension);

    B.rightCols(DOFsize) = identityDOFsizeDOFsize;
    Sc.leftCols(rank) = identityRankRank;

    qrDecomposition = Eigen::HouseholderQR<Eigen::MatrixXf>(DOFsize+floatingBaseDOF, CstrSpaceDimension);
	return true;
}

void ForceEstimator::updateHook() {
    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    in_jacobianCstr_flow = in_jacobianCstr_port.read(in_jacobianCstr_var);
    in_jacobianDotCstr_flow = in_jacobianDotCstr_port.read(in_jacobianDotCstr_var);
    in_inertia_flow = in_inertia_port.read(in_inertia_var);
	in_h_flow = in_h_port.read(in_h_var);
    in_torques_flow = in_torques_port.read(in_torques_var);

    if (in_robotstatus_flow == RTT::NoData
            || in_jacobianCstr_flow == RTT::NoData
            || in_jacobianDotCstr_flow == RTT::NoData
            || in_inertia_flow == RTT::NoData
            || in_h_flow == RTT::NoData
            || in_torques_flow == RTT::NoData) {
		return;
	}

    out_forceA_var = (in_jacobianCstr_var * in_inertia_var.inverse() * in_jacobianCstr_var.transpose() + tmpeyeCSdimCSdim).inverse()
                    * (in_jacobianCstr_var * in_inertia_var.inverse() * (in_h_var - B.transpose() * in_robotstatus_var.torques) - in_jacobianDotCstr_var * in_robotstatus_var.velocities);

    qrDecomposition.compute(in_jacobianCstr_var.transpose());
    Q = qrDecomposition.householderQ();// * thinEYE;
//    assert(Q.rows()==DOFsize+floatingBaseDOF);
//    assert(Q.cols()==DOFsize+floatingBaseDOF);
    R = qrDecomposition.matrixQR();
//    assert(R.rows()==DOFsize+floatingBaseDOF);
//    assert(R.cols()==CstrSpaceDimension);

    Rsmall = R.topRows(CstrSpaceDimension);
//    assert(Rsmall.rows()==CstrSpaceDimension);
//    assert(Rsmall.cols()==CstrSpaceDimension);

    //out_forceB_var = Rsmall.inverse() * Sc * Q.transpose() * (in_torques_var.torques - B.transpose() * in_robotstatus_var.torques);

    out_forceA_port.write(out_forceA_var);
    out_forceB_port.write(out_forceB_var);
}

void ForceEstimator::stopHook() {
}

void ForceEstimator::cleanupHook() {
	portsArePrepared = false;
}

void ForceEstimator::setDOFsize(unsigned int DOFsize) {
	assert(DOFsize > 0);
	this->DOFsize = DOFsize;
    identityDOFsizeDOFsize = Eigen::MatrixXf::Identity(DOFsize,DOFsize);
}

void ForceEstimator::setFloatingBaseMode(bool fbMode) {
    if (fbMode){
        floatingBaseDOF = 6;
    }
    else{
        floatingBaseDOF = 0;
    }
}

void ForceEstimator::setTaskSpaceDimension(const unsigned int TaskSpaceDimension) {
    this->TaskSpaceDimension = TaskSpaceDimension;
}

void ForceEstimator::setCstrSpaceDimension(const unsigned int CstrSpaceDimension) {
    this->CstrSpaceDimension = CstrSpaceDimension;
    rank = (CstrSpaceDimension/6);
    identityCSdimCSdim = Eigen::MatrixXf::Identity(CstrSpaceDimension,CstrSpaceDimension);
    tmpeyeCSdimCSdim = 0.001 * identityCSdimCSdim;
}

void ForceEstimator::preparePorts() {
	if (portsArePrepared) {
        ports()->removePort("in_robotstatus_port");
        ports()->removePort("in_jacobianCstr_port");
        ports()->removePort("in_jacobianDotCstr_port");
        ports()->removePort("in_inertia_port");
        ports()->removePort("in_h_port");
        ports()->removePort("in_torques_port");

        ports()->removePort("out_forceA_port");
        ports()->removePort("out_forceB_port");
	}

	//prepare input
    in_robotstatus_var = rstrt::robot::JointState(DOFsize);
	in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for the robot feedback");
	ports()->addPort(in_robotstatus_port);
	in_robotstatus_flow = RTT::NoData;

    in_jacobianCstr_var = Eigen::MatrixXf(CstrSpaceDimension, DOFsize);
    in_jacobianCstr_port.setName("in_jacobianCstr_port");
    in_jacobianCstr_port.doc("Input port for the jacobian");
    ports()->addPort(in_jacobianCstr_port);
    in_jacobianCstr_flow = RTT::NoData;

    in_jacobianDotCstr_var = Eigen::MatrixXf(CstrSpaceDimension, DOFsize);
    in_jacobianDotCstr_port.setName("in_jacobianDotCstr_port");
    in_jacobianDotCstr_port.doc("Input port for the derivative of jacobian");
    ports()->addPort(in_jacobianDotCstr_port);
    in_jacobianDotCstr_flow = RTT::NoData;

    in_inertia_var = Eigen::MatrixXf(TaskSpaceDimension,TaskSpaceDimension);
    in_inertia_port.setName("in_inertia_port");
    in_inertia_port.doc("Input port for inertia matrix");
    ports()->addPort(in_inertia_port);
    in_inertia_flow = RTT::NoData;

	in_h_var = Eigen::VectorXf(DOFsize);
	in_h_port.setName("in_h_port");
    in_h_port.doc("Input port for h vector");
	ports()->addPort(in_h_port);
	in_h_flow = RTT::NoData;

    in_torques_var = rstrt::dynamics::JointTorques(DOFsize);
    in_torques_port.setName("in_torques_port");
    in_torques_port.doc("Input port for torques vector");
    ports()->addPort(in_torques_port);
    in_torques_flow = RTT::NoData;


	//prepare output
    out_forceA_var = Eigen::VectorXf(CstrSpaceDimension);
    out_forceA_var.setZero();
    out_forceA_port.setName("out_forceA_port");
    out_forceA_port.doc("Output port for forceA vector");
    out_forceA_port.setDataSample(out_forceA_var);
    ports()->addPort(out_forceA_port);

    out_forceB_var = Eigen::VectorXf(CstrSpaceDimension);
    out_forceB_var.setZero();
    out_forceB_port.setName("out_forceB_port");
    out_forceB_port.doc("Output port for forceB vector");
    out_forceB_port.setDataSample(out_forceB_var);
    ports()->addPort(out_forceB_port);

	portsArePrepared = true;
}

void ForceEstimator::checkQR(
            Eigen::MatrixXf const & A,
            Eigen::MatrixXf const & Q,
            Eigen::MatrixXf const & R) {

    std::cout << "check QR decomposition with " << A << " and " << Q << " and " << R << std::endl;

    unsigned int m = A.rows();
    unsigned int n = A.cols();
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(m,n);

    std::cout << "i = " << this->getSumOfMatrix(A - Q * R) << std::endl;
    std::cout << "ii = " << this->getSumOfMatrix(Q.transpose() * Q - I) << std::endl;
    std::cout << "iii = " << this->getSumOfMatrix(Q.transpose() * A - R) << std::endl;
    std::cout << "iv = " << this->getSumOfMatrix(A * R.inverse() - Q) << std::endl;
}

float ForceEstimator::getSumOfMatrix(Eigen::MatrixXf const & mat) {
    if (mat.rows() == 0 || mat.cols() == 0){
        throw std::runtime_error("getSumOfMatrix => mat has zero rows or cols");
    }
    assert(mat.rows() > 0 && mat.cols() > 0);

    float sum = 0.0;
    for(unsigned int r=0; r<mat.rows(); r++){
        for(unsigned int c=r; c<mat.cols(); c++){
            sum = sum + mat(r,c);
        }
    }
    return sum;
}

void ForceEstimator::displayStatus() {
    RTT::log(RTT::Info) << "DOFsize " << DOFsize << RTT::endlog();
    RTT::log(RTT::Info) << "floatingBaseDOF " << floatingBaseDOF << RTT::endlog();
    RTT::log(RTT::Info) << "rank " << rank << RTT::endlog();
    RTT::log(RTT::Info) << "TaskSpaceDimension " << TaskSpaceDimension << RTT::endlog();
    RTT::log(RTT::Info) << "CstrSpaceDimension " << CstrSpaceDimension << RTT::endlog();

    RTT::log(RTT::Info) << "in_robotstatus_var.angles \n" << in_robotstatus_var.angles << RTT::endlog();
    RTT::log(RTT::Info) << "in_robotstatus_var.velocities \n" << in_robotstatus_var.velocities << RTT::endlog();
    RTT::log(RTT::Info) << "in_robotstatus_var.torques \n" << in_robotstatus_var.torques << RTT::endlog();
    RTT::log(RTT::Info) << "in_jacobianCstr_var \n" << in_jacobianCstr_var << RTT::endlog();
    RTT::log(RTT::Info) << "in_jacobianDotCstr_var \n" << in_jacobianDotCstr_var << RTT::endlog();
    RTT::log(RTT::Info) << "in_inertia_var \n" << in_inertia_var << RTT::endlog();
	RTT::log(RTT::Info) << "in_h_var \n" << in_h_var << RTT::endlog();
    RTT::log(RTT::Info) << "in_torques_var \n" << in_torques_var << RTT::endlog();
    RTT::log(RTT::Info) << "out_forceA_var \n" << out_forceA_var << RTT::endlog();
    RTT::log(RTT::Info) << "out_forceB_var \n" << out_forceB_var << RTT::endlog();

    RTT::log(RTT::Info) << "R \n" << R << RTT::endlog();
    RTT::log(RTT::Info) << "Rsmall \n" << Rsmall << RTT::endlog();
    RTT::log(RTT::Info) << "Rsmall.inverse() \n" << Rsmall.inverse() << RTT::endlog();
    RTT::log(RTT::Info) << "Q \n" << Q << RTT::endlog();
    RTT::log(RTT::Info) << "Sc \n" << Sc << RTT::endlog();
    RTT::log(RTT::Info) << "Rsmall.inverse() * Sc * Q.transpose() \n" << Rsmall.inverse() * Sc * Q.transpose() << RTT::endlog();

    this->checkQR(in_jacobianCstr_var.transpose(),Q,R);

}



//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(ForceEstimator)

