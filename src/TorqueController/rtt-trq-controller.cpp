#include <rtt/Component.hpp>
#include <iostream>
#include <nemo/Vector.h>
#include <nemo/Mapping.h>
#include <rtt/Activity.hpp>
#include "rtt-trq-controller.hpp"

#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace rci;

#define l(lvl) log(lvl) << "[" << this->getName() << "] "

bool once = true;

void incomingRCIJointAnglesFromRSB(rci::JointAnglesPtr jAngles) {
	boost::mutex::scoped_lock lock(rsbcmdJointAngles_mutex);
	{
		rsbcmdJointAngles_store->setValues(jAngles->asDoubleVector());
		rsbCMDJntPos_Flow = RTT::NewData;
	}
}

RTTTrqController::RTTTrqController(std::string const& name) :
		RTTArmControllerBase(name, "lwr_arm_base_link", "lwr_arm_7_link", 7),
		// Name, initial value
		cmdJntPos_Port("cmdJntPos", 0.0), cmdJntTrq_Port("cmdJntTrq", 0.0), currJntPos_Port(
				"currJntPos"), currJntVel_Port("currJntVel"), currJntTrq_Port(
				"currJntTrq"), cmdJntImp_Port("cmdJntImp"), gain(0.0), currJntPos_Flow(
				RTT::NoData), currJntVel_Flow(RTT::NoData), currJntTrq_Flow(
				RTT::NoData) {

	internalStartTime = 0.0;

	rsbCMDJntPos_Flow = RTT::NoData;

	this->ports()->addPort(cmdJntPos_Port).doc(
			"Sending joint position commands.");
	this->ports()->addPort(cmdJntTrq_Port).doc(
			"Sending joint torque commands.");

	this->ports()->addPort(currJntPos_Port).doc(
			"Receiving current joint position.");
	this->ports()->addPort(currJntVel_Port).doc(
			"Receiving current joint velocity.");
	this->ports()->addPort(currJntTrq_Port).doc(
			"Receiving current joint torque.");

	gain = 10.0;
	outJntPos = JointAngles::create(7, 0.5);
	rsbcmdJointAngles_store = outJntPos;
	cmdJntPos_Port.setDataSample(JointAngles::create(7, 0.5));

	outJntTrq = JointTorques::create(7, 0.0);
	cmdJntTrq_Port.setDataSample(JointTorques::create(7, 0.0));

	// set up listener to receive cmds from outside
	rsb::Factory& factory = rsb::getFactory();
	rsb::Scope scope("/rtt/lwr/cmd/position");
	rsbcmdJointPosition_Listener = factory.createListener(scope);
	rsbcmdJointPosition_Listener->addHandler(
			rsb::HandlerPtr(
					new rsb::DataFunctionHandler<rci::JointAngles>(
							&incomingRCIJointAnglesFromRSB)));

	currJntPos = rci::JointAngles::fromRad(nemo::RealVector(nemo::dim(7), 0.0));
	currJntVel = rci::JointVelocities::fromDeg_s(
			nemo::RealVector(nemo::dim(7), 0.0));
	currJntTrq = rci::JointTorques::fromNm(nemo::RealVector(nemo::dim(7), 0.0));

	this->addOperation("parseURDFforKDL", &RTTTrqController::parseURDFforKDL,
			this, OwnThread).doc("Parses a URDF string to create a KDL::Tree.").arg(
			"urdfString", "URDF string to parse.");

	l(Info) << "constructed !" << endlog();
}

bool RTTTrqController::configureHook() {

	initKDLTools();
	jnt_trq_cmd_.resize(kdl_chain_.getNrOfJoints());
	jnt_trq_cmd_Motion.resize(kdl_chain_.getNrOfJoints());
	jnt_trq_cmd_Nullspace.resize(kdl_chain_.getNrOfJoints());
	jnt_trq_cmd_Force.resize(kdl_chain_.getNrOfJoints());
	rsbCMDJntPos.resize(kdl_chain_.getNrOfJoints());
	kg_.resize(kdl_chain_.getNrOfJoints());
	kg_.setConstant(1.0);

	Lamda.resize(6, 6);
	h.resize(6, 1);
	F.resize(6, 1);

	yD.resize(6);
	yD(0) = 0.5;
	yD(1) = -0.5;
	yD(2) = 0;
	yD(3) = 0;
	yD(4) = 0;
	yD(5) = 0;

    Eigen::VectorXd init(7), final(7);
    Eigen::VectorXd Pi(6), Pf(6);
    init.setZero(7);
    final.setZero(7);
    Pi.resize(6);
    Pf.resize(6);

    Pi << 0.0, 0.0, 1.178, 0.0, 0.0, 0.0;
    Pf << -0.71, -0.23, 0.55, 0.0, 0.0, 0.0;



    final << 1.5708, 1.3963, 1.2217, 1.0472, 0.8727, 0.6981, 0.5236;
    start_time = 3.0;

    this->QP = QuinticPolynomial(this->start_time, start_time+30,init, final);
    this->_task_test = TaskTest(this->start_time, start_time+10,Pi, Pf);

//    _gravity.resize(DEFAULT_NR_JOINTS);
//    _inertia.resize(DEFAULT_NR_JOINTS);
//    _coriolis.resize(DEFAULT_NR_JOINTS);

    q_des_Nullspace.resize(DEFAULT_NR_JOINTS);
    q_des_Nullspace.data.setZero();
//    q_des_Nullspace(0) = 0.101942;
//    q_des_Nullspace(1) = 0.960698;
//	q_des_Nullspace(2) = -0.1172;
//	q_des_Nullspace(3) = 0.371442;
//	q_des_Nullspace(4) = 0.442573;
//	q_des_Nullspace(5) = 2.09441;
//	q_des_Nullspace(6) = 0.0884984;


    q_des_Nullspace(0) = 0.1809;
    q_des_Nullspace(1) =-0.2697;
	q_des_Nullspace(2) =-0.0999;
	q_des_Nullspace(3) =-1.2344;
	q_des_Nullspace(4) =-0.1026;
	q_des_Nullspace(5) = 1.1649;
	q_des_Nullspace(6) = 0.0;


    q_tmp.resize(DEFAULT_NR_JOINTS);
    qd_tmp.resize(DEFAULT_NR_JOINTS);
    qdd_tmp.resize(DEFAULT_NR_JOINTS);
    p_tmp.resize(6);
    pd_tmp.resize(6);
    pdd_tmp.resize(6);

    _lambda_des.resize(6);
    _lambda_des.setConstant(0.0);
	_lambda_des[0] = -5;

    rne_torques.resize(DEFAULT_NR_JOINTS);
    ext_force.resize(DEFAULT_NR_JOINTS);

    Kp_joint.resize(DEFAULT_NR_JOINTS);
    Kd_joint.resize(DEFAULT_NR_JOINTS);
//    Kp_joint.setConstant(2.0);
//	Kd_joint.setConstant(1.4);
    Kp_joint.setConstant(20.0);
	Kd_joint.setConstant(6.0);

    Kp_cart.resize(6);
    Kd_cart.resize(6);

    Kp_cart.setConstant(50.0);
    Kd_cart.setConstant(14.0);

    tau_0.resize(DEFAULT_NR_JOINTS);

    //Khatib controller:
	Lamda.resize(6,6);
	Lamda_cstr.resize(6,6);
	C_bar.resize(6);
	G_bar.resize(6);
	CG_bar.resize(6);
	Forces.resize(6);
	Forces_cstr.resize(6);

	lastJntVel = rci::JointVelocities::create(DEFAULT_NR_JOINTS, 0.0);

//	_jac.resize(DEFAULT_NR_JOINTS);
//	_jac_dot.resize(DEFAULT_NR_JOINTS);
	jac_cstr_.resize(6, DEFAULT_NR_JOINTS);
	jac_cstr_MPI.resize(DEFAULT_NR_JOINTS, 6);

	identity77.resize(7,7);
	identity66.resize(6,6);
	identity77 = Eigen::MatrixXd::Identity(7, 7);
	identity66 = Eigen::MatrixXd::Identity(6, 6);

	tmpeye77.resize(7,7);
	tmpeye66.resize(6,6);
	tmpeye77 = 0.01 * identity77;
	tmpeye66 = 0.01 * identity66;

	preLambda.resize(6,6);
	ref_acc.resize(6);

	P.resize(DEFAULT_NR_JOINTS,DEFAULT_NR_JOINTS);


	H_cstr_.resize(DEFAULT_NR_JOINTS,DEFAULT_NR_JOINTS);
	C_cstr_.resize(DEFAULT_NR_JOINTS);


    RTT::log(RTT::Error) << "qi :\n" << init.transpose() << RTT::endlog();
    RTT::log(RTT::Error) << "qf :\n" << final.transpose() << RTT::endlog();
    //EOP
	l(Info) << "configured !" << endlog();
	return true;
}

bool RTTTrqController::startHook() {
	if (!this->isConfigured()) {
		l(Error) << "Not configured yet: Needs to be configured first!"
				<< endlog();
		return false;
	}
	l(Info) << "started !" << endlog();
	internalStartTime = getSimulationTime();
	last_SimulationTime = getSimulationTime();
	return true;

}

double rtt_time_ = 0.0;

double RTTTrqController::getSimulationTime(){
    return 1E-9
            * RTT::os::TimeService::ticks2nsecs(
                    RTT::os::TimeService::Instance()->getTicks());
}

Eigen::VectorXd RTTTrqController::getQFromGazebo_EIGEN(){
    Eigen::VectorXd tmp_q(7);
    for (int i = 0; i < 7; i++) {
        tmp_q(i)  = currJntPos->radVector()[i];
    }
    return tmp_q;
}

Eigen::VectorXd RTTTrqController::getQdFromGazebo_EIGEN(){
    Eigen::VectorXd tmp_qd(7);
    for (int i = 0; i < 7; i++) {
        tmp_qd(i)  = currJntVel->rad_sVector()[i];
    }
    return tmp_qd;
}

KDL::JntArray   RTTTrqController::getQFromGazebo_KDL(){
    KDL::JntArray q_from_robot;
    q_from_robot.data = getQFromGazebo_EIGEN();
    return q_from_robot;
}

KDL::JntArray   RTTTrqController::getQdFromGazebo_KDL(){
    KDL::JntArray qd_from_robot;
    qd_from_robot.data = getQdFromGazebo_EIGEN();
    return qd_from_robot;
}

//Eigen::MatrixXd RTTTrqController::inverseDynamicsTorques(){
//    //This works based on the very bad idea of having "many global variables". Dependancy injection is need.
//    //perhaps, we should make a container for dynamic model...
//    return _inertia.data*(qdd_tmp.data-Kp_joint.asDiagonal()*(q_from_robot.data - q_tmp.data)-Kd_joint.asDiagonal()*(qd_from_robot.data-qd_tmp.data))
//                 +_coriolis.data
//                 +_gravity.data;

//}

void RTTTrqController::updateHook() {
	/** Read feedback from robot */
//

	// check if port is connected
	if (currJntPos_Port.connected()) {
		// read into "currJntPos" and save state of data into "currJntPos_Flow", which can be "NewData", "OldData" or "NoData".
		currJntPos_Flow = currJntPos_Port.read(currJntPos);
	}
	if (currJntVel_Port.connected()) {
		currJntVel_Flow = currJntVel_Port.read(currJntVel);
	}
	if (currJntTrq_Port.connected()) {
		currJntTrq_Flow = currJntTrq_Port.read(currJntTrq);
	}

	// check for NoData
	if ((currJntPos_Flow == RTT::NoData) || (currJntVel_Flow == RTT::NoData)
			|| (currJntTrq_Flow == RTT::NoData)) {
		// skip this step, because we don't receive all the necessary data.
		return;
	}

	double delta_t = getSimulationTime() - last_SimulationTime;
	currJntAcc = rci::JointAccelerations::fromRad_ss( (lastJntVel->rad_sVector() - currJntVel->rad_sVector()) * (1 / delta_t) );

	// calculate mass(H), G, jac_ (based on velocities)
    updateDynamicsAndKinematics(currJntPos, currJntVel, currJntTrq);
    jac_cstr_ = jac_.data; //TODO
    jac_cstr_.row(0).setZero();
    jac_cstr_.row(3).setZero();
    jac_cstr_.row(4).setZero();
    jac_cstr_.row(5).setZero();


	// read rsb position command
	{
		boost::mutex::scoped_try_lock lock(rsbcmdJointAngles_mutex,
				boost::try_to_lock);
		if (lock) {
			p.convertRealVectorToEigenVectorXd(
					rsbcmdJointAngles_store->asDoubleVector(), rsbCMDJntPos);
		}
	}
	// check if command from rsb is new
	if (rsbCMDJntPos_Flow == RTT::NewData) {
		l(Info) << "New command from RSB has arrived!" << endlog();
		rsbCMDJntPos_Flow = RTT::OldData;
	}

	//
	/** INSERT TORQUE or POSITION CONTROLLER HERE */
	//
    if (getSimulationTime()-internalStartTime > start_time) {

        //getting current time
        double t = getSimulationTime();

        //init
//        _jac_dot.data.setZero();
//		_jac.data.setZero();
//        q_from_robot = getQFromGazebo_KDL();
//        qd_from_robot = getQdFromGazebo_KDL();


        //getting temps (=desired values) FOR JOINT TRAJECOTRY:
        q_tmp.data   = QP.getQ(t);
        qd_tmp.data  = QP.getQd(t);
        qdd_tmp.data = QP.getQdd(t);

        //getting temps (=desired values) FOR ENDEFFECTOR TRAJECOTRY:
        p_tmp.data   = _task_test.getPosition(t);
        pd_tmp.data  = _task_test.getVelocity(t);
        pdd_tmp.data = _task_test.getAcceleration(t);

        // start open loop joint controller
//        joint_position_velocity_des.q     = q_tmp;
//		joint_position_velocity_des.qdot  = qd_tmp;
//
//        id_dyn_solver->JntToMass(joint_position_velocity_des.q, H_);
//		id_dyn_solver->JntToGravity(joint_position_velocity_des.q, G_);
//		id_dyn_solver->JntToCoriolis(joint_position_velocity_des.q, joint_position_velocity_des.qdot, C_);
        // stop open loop joint controller

//
//        //compute inertia, coriolis and gravity
//        id_dyn_solver->JntToMass(q_from_robot, _inertia);
//        id_dyn_solver->JntToCoriolis(q_from_robot, qd_from_robot, _coriolis);
//        id_dyn_solver->JntToGravity(q_from_robot, _gravity);
//
//        //compute jacobian
//        jnt_to_jac_solver->JntToJac(q_from_robot, _jac, kdl_chain_.getNrOfSegments());
//        jnt_to_jac_dot_solver->JntToJacDot(joint_position_velocity, _jac_dot, kdl_chain_.getNrOfSegments());
//
//
//        //compute velocity and position of EE
//        jnt_to_cart_pos_solver->JntToCart(joint_position_velocity.q, cartFrame, kdl_chain_.getNrOfSegments());
//        jnt_to_cart_vel_solver->JntToCart(joint_position_velocity, velFrame, kdl_chain_.getNrOfSegments());


        //convert current endeffector pose to eigen
        Eigen::VectorXd _curr_ee_pose(6), _curr_ee_vel(6);
        _curr_ee_pose << cartFrame.p.x(), cartFrame.p.y(), cartFrame.p.z(), cartFrame.M.GetRot().x(), cartFrame.M.GetRot().y(), cartFrame.M.GetRot().z();
        _curr_ee_vel  << velFrame.p.v.x(), velFrame.p.v.y(), velFrame.p.v.z(), velFrame.GetFrame().M.GetRot().x(), velFrame.GetFrame().M.GetRot().y(), velFrame.GetFrame().M.GetRot().z();
//        _curr_ee_pose << cartFrame.p.x(), cartFrame.p.y(), cartFrame.p.z(), 0.0, 0.0, 0.0;
//        _curr_ee_vel  << velFrame.p.v.x(), velFrame.p.v.y(), velFrame.p.v.z(), 0.0, 0.0, 0.0;

//        l(Error) << "pose: " << _curr_ee_pose << RTT::endlog();
//        l(Error) << "velo: " << _curr_ee_vel << RTT::endlog();
//
//        l(Error) << "cartFrame.M.GetRot().x(): " << cartFrame.M.GetRot().x() / (2*M_PI) * 360 << RTT::endlog();
//        l(Error) << "cartFrame.M.GetRot().y(): " << cartFrame.M.GetRot().y() / (2*M_PI) * 360<< RTT::endlog();
//        l(Error) << "cartFrame.M.GetRot().z(): " << cartFrame.M.GetRot().z() / (2*M_PI) * 360 << RTT::endlog();
//

//        l(Error) << "jnt_pos_: " << jnt_pos_ << RTT::endlog();

//        throw "out";

        // start open loop joint controller
//        jnt_trq_cmd_ = H_.data*(qdd_tmp.data-Kp_joint.asDiagonal()*(jnt_pos_ - q_tmp.data)-Kd_joint.asDiagonal()*(jnt_vel_-qd_tmp.data)) + C_.data + G_.data;
//        jnt_trq_cmd_ = H_.data*(qdd_tmp.data) + C_.data + G_.data;
        // stop open loop joint controller





        //Start Khatib endeffector motion controller -> dennis variables
        H_.data = H_.data + tmpeye77; // add regression for better inverse computation
//        Lamda = (jac_.data * H_.data.inverse() * jac_.data.transpose()).inverse();
        Lamda = (jac_.data * H_.data.inverse() * jac_.data.transpose() + tmpeye66).inverse(); //add regression for better inverse computation
        h = C_.data + G_.data;
        CG_bar = Lamda*(jac_.data * H_.data.inverse() * (h) - jac_dot_.data * jnt_vel_);

        ref_acc = pdd_tmp.data + Kd_cart.asDiagonal()*(pd_tmp.data - _curr_ee_vel) + Kp_cart.asDiagonal()*(p_tmp.data - _curr_ee_pose);
        Forces  = Lamda * ref_acc + CG_bar;
        jnt_trq_cmd_Motion = jac_.data.transpose()*Forces;
        //Stop Khatib endeffector motion controller


        //Start Khatib nullspace controller
        tau_0 = Kp_joint.asDiagonal()*(q_des_Nullspace.data - jnt_pos_) - Kd_joint.asDiagonal()*(jnt_vel_);
        N = identity77 - jac_.data.transpose() * ( Lamda * jac_.data * H_.data );
//        N = identity77 - jac_.data.transpose() * ( jac_.data );
        jnt_trq_cmd_Nullspace = N * tau_0;

//        jnt_trq_cmd_Nullspace.setZero();
		//Stop Khatib nullspace controller

//        jnt_trq_cmd_ = jnt_trq_cmd_Motion;
        jnt_trq_cmd_ = jnt_trq_cmd_Motion + 0.1 * jnt_trq_cmd_Nullspace;

//        l(Error) << "tau_0: " << tau_0 << RTT::endlog();
//        l(Error) << "jnt_trq_cmd_Motion: " << jnt_trq_cmd_Motion << RTT::endlog();
//        l(Error) << "jnt_trq_cmd_Nullspace: " << jnt_trq_cmd_Nullspace << RTT::endlog();
//        throw "out";

        //compute constrained projection
//        jac_cstr_MPI = (jac_cstr_.transpose() * jac_cstr_).inverse() * jac_cstr_.transpose();
//        P = identity77 - (jac_cstr_MPI * jac_cstr_);
//        H_cstr_ = P * H_.data +  identity77 - P;
//		C_cstr_ = -(jac_cstr_MPI * jac_cstr_);
//		Lamda_cstr = (jac_.data * H_cstr_.inverse() * P * jac_.data.transpose()).inverse(); //equals Lamda in paper => + tmpeye66


        //Start Khatib projected endeffector motion controller
//        ref_acc = pdd_tmp.data + Kd_cart.asDiagonal()*(pd_tmp.data - _curr_ee_vel) + Kp_cart.asDiagonal()*(p_tmp.data - _curr_ee_pose);
//        h = C_.data + G_.data;
//        Forces_cstr = Lamda_cstr * ref_acc + Lamda_cstr * (jac_.data * H_cstr_.inverse() * P * h - (jac_dot_.data + jac_.data * H_cstr_.inverse() * C_cstr_)*jnt_vel_ );
//        jnt_trq_cmd_ = P * jac_.data.transpose()*Forces_cstr;
        //Stop Khatib projected endeffector motion controller




        //Start constrained nullspace controller
//        N = identity77 - jac_.data.transpose() * ((jac_.data * H_cstr.data.inverse() * P * jac_.data.transpose()).inverse() * jac_.data * H_cstr.data.inverse() * P);
//
//        tau_0 = Kp_joint.asDiagonal()*(q_tmp.data - jnt_pos_) - Kd_joint.asDiagonal()*(jnt_vel_) ;
//        jnt_trq_cmd_ += P * N * tau_0;

        //Stop constrained nullspace controller


        //Start external forces controller
//        jnt_trq_cmd_ = (identity77 - P) * (h) + (identity77 - P) * H_.data * H_cstr_.inverse() * (P * H_.data * currJntAcc * + C_cstr) + jac_cstr_.transpose() * _lambda_des;

        //Stop external forces controller



        /* Inverse dynamic controller M+C+G=t. This controller works. DON"T TOUCH!
        jnt_trq_cmd_ = _inertia.data*(qdd_tmp.data-Kp_joint.asDiagonal()*(q_from_robot.data - q_tmp.data)-Kd_joint.asDiagonal()*(qd_from_robot.data-qd_tmp.data))
                     +_coriolis.data
                     +_gravity.data;
        */

//        RTT::log(RTT::Error) << "qdd :\n" << qdd_tmp.data<< RTT::endlog();

	} else {
        kg_.setConstant(1); // heavily overshooting!!!! BEWARE
        jnt_trq_cmd_ = kg_.asDiagonal() * G_.data;
	}

//    RTT::log(RTT::Error) << "jnt_trq_cmd_ :\n" << jnt_trq_cmd_ << RTT::endlog();

    Eigen::VectorXd tmp;
    tmp.resize(7);
    for (int i = 0; i < 7; i++) {
        tmp(i) = currJntPos->radVector()[i];
    }



    // DON'T TOUCH BELOW
	// convert eigen back to rci
	for (int i = 0; i < outJntTrq->getDimension(); i++) {
		outJntTrq->setFromNm(i, jnt_trq_cmd_.data()[i]);
	}

//	l(Warning) << "outJntTrq: " << outJntTrq->print() << endlog();
//	l(Warning) << "G_:\n" << G_ << endlog();

	// write torques to robot
	if (cmdJntTrq_Port.connected()) {
		cmdJntTrq_Port.write(outJntTrq);
	}

	last_SimulationTime = getSimulationTime();
	lastJntVel = currJntVel;
}

void RTTTrqController::stopHook() {
	l(Info) << "executes stopping !" << endlog();
}

void RTTTrqController::cleanupHook() {
	initKDLTools();
	l(Info) << "cleaning up !" << endlog();
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(RTTController)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_LIST_COMPONENT_TYPE(RTTTrqController)
