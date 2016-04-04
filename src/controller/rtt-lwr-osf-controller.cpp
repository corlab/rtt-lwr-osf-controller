#include <rtt/Component.hpp>
#include <iostream>
#include <nemo/Vector.h>
#include <nemo/Mapping.h>
#include <rtt/Activity.hpp>
#include "rtt-lwr-osf-controller.hpp"

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

RttLwrOSFController::RttLwrOSFController(std::string const& name) :
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

	this->addOperation("parseURDFforKDL", &RttLwrOSFController::parseURDFforKDL,
			this, OwnThread).doc("Parses a URDF string to create a KDL::Tree.").arg(
			"urdfString", "URDF string to parse.");


	// resize gains and use
	tenGains.resize(10);

	//set gain default values if you like
//    Kp_cartTranslationKhatibGain = 50;
//    Kd_cartTranslationKhatibGain = 14;
//    Kp_cartOrientationKhatibGain = 2500;
//    Kd_cartOrientationKhatibGain = 100;
//    Kp_jointKhatibGain = 20;
//    Kd_jointKhatibGain = 6;

	this->addProperty( "use_original_khatib_controller", use_original_khatib_controller ).doc("use_original_khatib_controller Example Description");
	this->addProperty( "use_euler_orientation", use_euler_orientation ).doc("use_euler_orientation Example Description");

	this->addProperty( "Kp_cartTranslationKhatibGain", Kp_cartTranslationKhatibGain ).doc("KhatibGain1 Example Description");
	this->addProperty( "Kd_cartTranslationKhatibGain", Kd_cartTranslationKhatibGain ).doc("KhatibGain2 Example Description");
	this->addProperty( "Kp_cartOrientationEulerKhatibGain", Kp_cartOrientationEulerKhatibGain ).doc("EulerKhatibGain3 Example Description");
	this->addProperty( "Kd_cartOrientationEulerKhatibGain", Kd_cartOrientationEulerKhatibGain ).doc("EulerKhatibGain4 Example Description");
	this->addProperty( "Kp_cartOrientationQuaternionKhatibGain", Kp_cartOrientationQuaternionKhatibGain ).doc("QuaternionKhatibGain3 Example Description");
	this->addProperty( "Kd_cartOrientationQuaternionKhatibGain", Kd_cartOrientationQuaternionKhatibGain ).doc("QuaternionKhatibGain4 Example Description");
	this->addProperty( "Kp_jointKhatibGain", Kp_jointKhatibGain ).doc("KhatibGain5 Example Description");
	this->addProperty( "Kd_jointKhatibGain", Kd_jointKhatibGain ).doc("KhatibGain6 Example Description");

	this->addProperty( "Kp_cartTranslationConstrainedGain", Kp_cartTranslationConstrainedGain ).doc("ConstrainedGain1 Example Description");
	this->addProperty( "Kd_cartTranslationConstrainedGain", Kd_cartTranslationConstrainedGain ).doc("ConstrainedGain2 Example Description");
	this->addProperty( "Kp_cartOrientationEulerConstrainedGain", Kp_cartOrientationEulerConstrainedGain ).doc("EulerConstrainedGain3 Example Description");
	this->addProperty( "Kd_cartOrientationEulerConstrainedGain", Kd_cartOrientationEulerConstrainedGain ).doc("EulerConstrainedGain4 Example Description");
	this->addProperty( "Kp_cartOrientationQuaternionConstrainedGain", Kp_cartOrientationQuaternionConstrainedGain ).doc("QuaternionConstrainedGain3 Example Description");
	this->addProperty( "Kd_cartOrientationQuaternionConstrainedGain", Kd_cartOrientationQuaternionConstrainedGain ).doc("QuaternionConstrainedGain4 Example Description");
	this->addProperty( "Kp_jointConstrainedGain", Kp_jointConstrainedGain ).doc("ConstrainedGain5 Example Description");
	this->addProperty( "Kd_jointConstrainedGain", Kd_jointConstrainedGain ).doc("ConstrainedGain6 Example Description");

	this->addProperty( "tenGains", tenGains ).doc("tenGains Example Description");

	l(Info) << "constructed !" << endlog();
}

bool RttLwrOSFController::configureHook() {

	initKDLTools();

	// start test printout of the gains (can be removed)
	l(Error) << "use_original_khatib_controller: " << use_original_khatib_controller << endlog();
	l(Error) << "use_euler_orientation: " << use_euler_orientation << endlog();

	if (use_original_khatib_controller){
		l(Error) << "Kp_cartTranslationKhatibGain: " << Kp_cartTranslationKhatibGain << endlog();
		l(Error) << "Kd_cartTranslationKhatibGain: " << Kd_cartTranslationKhatibGain << endlog();
		if (use_euler_orientation){
			l(Error) << "Kp_cartOrientationEulerKhatibGain: " << Kp_cartOrientationEulerKhatibGain << endlog();
			l(Error) << "Kd_cartOrientationEulerKhatibGain: " << Kd_cartOrientationEulerKhatibGain << endlog();
		}
		else{
			l(Error) << "Kp_cartOrientationQuaternionKhatibGain: " << Kp_cartOrientationQuaternionKhatibGain << endlog();
			l(Error) << "Kd_cartOrientationQuaternionKhatibGain: " << Kd_cartOrientationQuaternionKhatibGain << endlog();
		}
		l(Error) << "Kp_jointKhatibGain: " << Kp_jointKhatibGain << endlog();
		l(Error) << "Kd_jointKhatibGain: " << Kd_jointKhatibGain << endlog();
	}
	else{
		l(Error) << "Kp_cartTranslationConstrainedGain: " << Kp_cartTranslationConstrainedGain << endlog();
		l(Error) << "Kd_cartTranslationConstrainedGain: " << Kd_cartTranslationConstrainedGain << endlog();
		if (use_euler_orientation){
			l(Error) << "Kp_cartOrientationEulerConstrainedGain: " << Kp_cartOrientationEulerConstrainedGain << endlog();
			l(Error) << "Kd_cartOrientationEulerConstrainedGain: " << Kd_cartOrientationEulerConstrainedGain << endlog();
		}
		else{
			l(Error) << "Kp_cartOrientationQuaternionConstrainedGain: " << Kp_cartOrientationQuaternionConstrainedGain << endlog();
			l(Error) << "Kd_cartOrientationQuaternionConstrainedGain: " << Kd_cartOrientationQuaternionConstrainedGain << endlog();
		}
		l(Error) << "Kp_jointConstrainedGain: " << Kp_jointConstrainedGain << endlog();
		l(Error) << "Kd_jointConstrainedGain: " << Kd_jointConstrainedGain << endlog();
	}
//	for (int i = 0; i < tenGains.size(); i++) {
//		l(Error) << "tenGains[" << i << "]: " << tenGains[i] << endlog();
//	}
	// stop test printout of the gains (can be removed)

	jointPosLimits_max.resize(DEFAULT_NR_JOINTS);
	jointPosLimits_min.resize(DEFAULT_NR_JOINTS);
	jointVelLimits_max.resize(DEFAULT_NR_JOINTS);
	jointVelLimits_min.resize(DEFAULT_NR_JOINTS);
	jointPosLimits_max << 	+2.96705972839,
							+2.09439510239,
							+2.96705972839,
							+2.09439510239,
							+2.96705972839,
							+2.09439510239,
							+2.96705972839;
	jointPosLimits_min  << 	-2.96705972839,
							-2.09439510239,
							-2.96705972839,
							-2.09439510239,
							-2.96705972839,
							-2.09439510239,
							-2.96705972839;

	safety_margin_Pos = 0.1;
	jointPosLimits_range = jointPosLimits_max - jointPosLimits_min;
	jointPosCritic_max = jointPosLimits_max - safety_margin_Pos*jointPosLimits_range;
	jointPosCritic_min = jointPosLimits_min + safety_margin_Pos*jointPosLimits_range;

	jointVelLimits_max << 	+1.91986217719,
							+1.91986217719,
							+2.26892802759,
							+2.26892802759,
							+2.26892802759,
							+3.14159265359,
							+3.14159265359;
	jointVelLimits_min = (-1) * jointVelLimits_max;

	safety_margin_Vel = 0.1;
	jointVelLimits_range = jointVelLimits_max - jointVelLimits_min;
	jointVelCritic_max = jointVelLimits_max - safety_margin_Vel*jointVelLimits_range;
	jointVelCritic_min = jointVelLimits_min + safety_margin_Vel*jointVelLimits_range;

	jnt_trq_cmd_.resize(kdl_chain_.getNrOfJoints());
	jnt_trq_cmd_Motion_Khatib.resize(kdl_chain_.getNrOfJoints());
	jnt_trq_cmd_Nullspace_Khatib.resize(kdl_chain_.getNrOfJoints());
	jnt_trq_cmd_Force_Khatib.resize(kdl_chain_.getNrOfJoints());
	jnt_trq_cmd_Motion_Projected.resize(kdl_chain_.getNrOfJoints());
	jnt_trq_cmd_Nullspace_Projected.resize(kdl_chain_.getNrOfJoints());
	jnt_trq_cmd_Force_Projected.resize(kdl_chain_.getNrOfJoints());
	rsbCMDJntPos.resize(kdl_chain_.getNrOfJoints());
	kg_.resize(kdl_chain_.getNrOfJoints());
	kg_.setConstant(1.0);

	Lamda.resize(6, 6);
	h.resize(6, 1);

	yD.resize(6);
	yD(0) = 0.5;
	yD(1) = -0.5;
	yD(2) = 0;
	yD(3) = 0;
	yD(4) = 0;
	yD(5) = 0;



    curr_ee_pose.resize(6);
    curr_ee_vel.resize(6);
    curr_ee_poseTranslation.resize(3);
    curr_ee_velTranslation.resize(3);
    curr_ee_poseOrientation.resize(3);
    curr_ee_velOrientation.resize(3);

    q_des_Nullspace.resize(DEFAULT_NR_JOINTS); // points endeffector to center of circle
    q_des_Nullspace.data.setZero();
    q_des_FirstPoint.resize(DEFAULT_NR_JOINTS); // points endeffector to first point of circle
    q_des_FirstPoint.data.setZero();

    //desired joint configuration for board 45 degrees and center of circle (= translation): [-0.45; 0.0; 0.75]
//    q_des_Nullspace.data(0) = 0.1809;
//    q_des_Nullspace.data(1) =-0.2697;
//	q_des_Nullspace.data(2) =-0.0999;
//	q_des_Nullspace.data(3) =-1.2344;
//	q_des_Nullspace.data(4) =-0.1026;
//	q_des_Nullspace.data(5) = 1.1649;
//	q_des_Nullspace.data(6) = 0.0;

    //desired joint configuration for board parallel to floor and center of circle (= translation): [-0.55; 0.0; 0.5]
	q_des_Nullspace.data(0) = 0.0410;
	q_des_Nullspace.data(1) = 0.0606;
	q_des_Nullspace.data(2) =-0.0154;
	q_des_Nullspace.data(3) =-1.3954;
	q_des_Nullspace.data(4) =-0.0548;
	q_des_Nullspace.data(5) = 1.1377;
	q_des_Nullspace.data(6) = 0.0;

    //desired joint configuration for board parallel to floor and first point of circle
    // q_des_FirstPoint.data(0) = 0.0974;
    // q_des_FirstPoint.data(1) = 0.0534;
    // q_des_FirstPoint.data(2) = 0.0872;
    // q_des_FirstPoint.data(3) =-1.7562;
    // q_des_FirstPoint.data(4) =-0.0325;
    // q_des_FirstPoint.data(5) = 1.2971;
    // q_des_FirstPoint.data(6) = 0.0;

    q_des_FirstPoint.data(0) = -0.1192;
    q_des_FirstPoint.data(1) = 0.3114;
    q_des_FirstPoint.data(2) = -0.1868;
    q_des_FirstPoint.data(3) = -1.5733;
    q_des_FirstPoint.data(4) = 0.1194;
    q_des_FirstPoint.data(5) = 1.0685;
    q_des_FirstPoint.data(6) = 0.0;

//lwr_arm_6_joint = -0.3255



    task_q.resize(DEFAULT_NR_JOINTS);
    task_qd.resize(DEFAULT_NR_JOINTS);
    task_qdd.resize(DEFAULT_NR_JOINTS);
    task_p.resize(6);
    task_pd.resize(6);
    task_pdd.resize(6);
    task_pTranslation.resize(3);
	task_pdTranslation.resize(3);
	task_pddTranslation.resize(3);
	task_pOrientation.resize(3);
	task_pdOrientation.resize(3);
	task_pddOrientation.resize(3);

    //start variables for quaternion feedback stuff
	desiredCartOrientationQuaternionU.resize(3);
	currCartOrientationQuaternionU.resize(3);
	QuaternionProductU.resize(3);
    QuaternionProductEuler.resize(3);
    //stop variables for quaternion feedback stuff

    lambda_des.resize(6);
    lambda_des.setConstant(0.0);
	lambda_des[2] = -5; //desired endeffector force, use z-axis here

    rne_torques.resize(DEFAULT_NR_JOINTS);
    ext_force.resize(DEFAULT_NR_JOINTS);

    Kp_cartTranslation.resize(3);
    Kd_cartTranslation.resize(3);
    Kp_cartOrientationEuler.resize(3);
    Kd_cartOrientationEuler.resize(3);
    Kp_cartOrientationQuaternion.resize(3);
	Kd_cartOrientationQuaternion.resize(3);
    Kp_joint.resize(DEFAULT_NR_JOINTS);
    Kd_joint.resize(DEFAULT_NR_JOINTS);

    if (use_original_khatib_controller){
		Kp_cartTranslation.setConstant(Kp_cartTranslationKhatibGain);
		Kd_cartTranslation.setConstant(Kd_cartTranslationKhatibGain);
		Kp_cartOrientationEuler.setConstant(Kp_cartOrientationEulerKhatibGain);
		Kd_cartOrientationEuler.setConstant(Kd_cartOrientationEulerKhatibGain);
		Kp_cartOrientationQuaternion.setConstant(Kp_cartOrientationQuaternionKhatibGain);
		Kd_cartOrientationQuaternion.setConstant(Kd_cartOrientationQuaternionKhatibGain);
		Kp_joint.setConstant(Kp_jointKhatibGain);
		Kd_joint.setConstant(Kd_jointKhatibGain);
    }
    else{
		Kp_cartTranslation.setConstant(Kp_cartTranslationConstrainedGain);
		Kd_cartTranslation.setConstant(Kd_cartTranslationConstrainedGain);
		Kp_cartOrientationEuler.setConstant(Kp_cartOrientationEulerConstrainedGain);
		Kd_cartOrientationEuler.setConstant(Kd_cartOrientationEulerConstrainedGain);
		Kp_cartOrientationQuaternion.setConstant(Kp_cartOrientationQuaternionConstrainedGain);
		Kd_cartOrientationQuaternion.setConstant(Kd_cartOrientationQuaternionConstrainedGain);
		Kp_joint.setConstant(Kp_jointConstrainedGain);
		Kd_joint.setConstant(Kd_jointConstrainedGain);
    }

    tau_0.resize(DEFAULT_NR_JOINTS);

    lastJntVel.resize(DEFAULT_NR_JOINTS);
	currJntAcc.resize(DEFAULT_NR_JOINTS);

    //Khatib controller:
	Lamda.resize(6,6);
	Lamda_cstr.resize(6,6);
	CG_bar.resize(6);
	Forces.resize(6);
	Forces_cstr.resize(6);

	jac_cstr_.resize(6, DEFAULT_NR_JOINTS);
	jac_cstr_MPI.resize(DEFAULT_NR_JOINTS, 6);

	identity77.resize(7,7);
	identity66.resize(6,6);
	identity33.resize(3,3);
	identity77 = Eigen::MatrixXd::Identity(7, 7);
	identity66 = Eigen::MatrixXd::Identity(6, 6);
	identity33 = Eigen::MatrixXd::Identity(3, 3);

	diagonal66.resize(6,6);
	diagonal33.resize(3,3);
	diagonal66 = identity66.diagonal().asDiagonal();
	diagonal33 = identity33.diagonal().asDiagonal();


	tmpeye77.resize(7,7);
	tmpeye66.resize(6,6);
	tmpeye77 = 0.01 * identity77;
	tmpeye66 = 0.01 * identity66;

	preLambda.resize(6,6);
	ref_acc.resize(6);
	ref_accTranslation.resize(3);
	ref_accOrientation.resize(3);

	P.resize(DEFAULT_NR_JOINTS,DEFAULT_NR_JOINTS);
	M_cstr_.resize(DEFAULT_NR_JOINTS,DEFAULT_NR_JOINTS);
	C_cstr_.resize(DEFAULT_NR_JOINTS,DEFAULT_NR_JOINTS);

	internalStartTime = getSimulationTime();
	last_SimulationTime = getSimulationTime();

    init.resize(DEFAULT_NR_JOINTS);
    final.resize(DEFAULT_NR_JOINTS);
    init.setZero(7);
    final.setZero(7);
    Pi.resize(6);
    Pf.resize(6);
    Pi << 0.0, 0.0, 1.178, 0.0, 0.0, 0.0;
    Pf << -0.71, -0.23, 0.55, 0.0, 0.0, 0.0;
    final << 1.5708, 1.3963, 1.2217, 1.0472, 0.8727, 0.6981, 0.5236;
    start_time = 5.0;
//    this->QP = QuinticPolynomial(this->start_time, start_time+30,init, final);
//    this->_task_test = TaskTest(this->start_time, start_time+10,Pi, Pf);
    this->cart_task = CartesianSpace_CircularTask(this->start_time, 0.6);

    std::string fname = "desired_cart_pose";
    this->csv_logger = new FileWriterCSV(fname);
    this->orientation_helper = OrientationHelper();


    //Start TEST
	cart_task.getPositionOrientation(start_time, task_pOrientation);

	l(Error) << "nemo stuff ####################################" << endlog();

    desiredCartOrientation = rci::Orientation::fromEulerAngles( task_pOrientation(0), task_pOrientation(1), task_pOrientation(2) );
    l(Error) << "task_pOrientation " << task_pOrientation << endlog();

	desiredCartOrientationQuaternionV    = desiredCartOrientation->q0();
	desiredCartOrientationQuaternionU(0) = desiredCartOrientation->q1();
	desiredCartOrientationQuaternionU(1) = desiredCartOrientation->q2();
	desiredCartOrientationQuaternionU(2) = desiredCartOrientation->q3();
	l(Error) << "desiredCartOrientationQuaternionV " << desiredCartOrientationQuaternionV << endlog();
	l(Error) << "desiredCartOrientationQuaternionU " << desiredCartOrientationQuaternionU << endlog();

	desiredCartOrientation = rci::Orientation::fromQuaternion(desiredCartOrientationQuaternionV, desiredCartOrientationQuaternionU(0), desiredCartOrientationQuaternionU(1), desiredCartOrientationQuaternionU(2) );
	QuaternionProductEuler(0) = desiredCartOrientation->a();
	QuaternionProductEuler(1) = desiredCartOrientation->b();
	QuaternionProductEuler(2) = desiredCartOrientation->c();
	l(Error) << "QuaternionProductEuler " << QuaternionProductEuler << endlog();

	l(Error) << "my stuff ####################################" << endlog();

	desiredCartOrientation = rci::Orientation::fromEulerAngles( task_pOrientation(0), task_pOrientation(1), task_pOrientation(2) );
	l(Error) << "task_pOrientation " << task_pOrientation << endlog();
	orientation_helper.Euler2Quaternion(task_pOrientation, desiredCartOrientationQuaternionV, desiredCartOrientationQuaternionU);
	l(Error) << "desiredCartOrientationQuaternionV " << desiredCartOrientationQuaternionV << endlog();
	l(Error) << "desiredCartOrientationQuaternionU " << desiredCartOrientationQuaternionU << endlog();

	orientation_helper.Quaternion2Euler(desiredCartOrientationQuaternionV, desiredCartOrientationQuaternionU, QuaternionProductEuler);
	l(Error) << "QuaternionProductEuler " << QuaternionProductEuler << endlog();

	//END TEST

    //EOP
	l(Info) << "configured !" << endlog();
	return true;
}

bool RttLwrOSFController::startHook() {
	if (!this->isConfigured()) {
		l(Error) << "Not configured yet: Needs to be configured first!"
				<< endlog();
		return false;
	}
	l(Info) << "started !" << endlog();
	return true;

}

double rtt_time_ = 0.0;

double RttLwrOSFController::getSimulationTime(){
    return 1E-9
            * RTT::os::TimeService::ticks2nsecs(
                    RTT::os::TimeService::Instance()->getTicks());
}


void RttLwrOSFController::updateHook() {
//	throw "out";

	/** Read feedback from robot */

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
    currJntAcc = (jnt_vel_ - lastJntVel) / 0.001;//delta_t ;

	// calculate mass(M_), coriolis(C_), gravity(G_), jacobian(jac_) (based on velocities)
    updateDynamicsAndKinematics(currJntPos, currJntVel, currJntTrq);
    M_.data = M_.data + tmpeye77; // add regularization for better inverse computation

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


        //getting desired values FOR JOINT TRAJECOTRY:
//        task_q.data   = QP.getQ(t-internalStartTime);
//        task_qd.data  = QP.getQd(t-internalStartTime);
//        task_qdd.data = QP.getQdd(t-internalStartTime);

        //getting desired values FOR ENDEFFECTOR TRAJECOTRY:
        cart_task.getPosition(t-internalStartTime, task_p.data);
        cart_task.getVelocity(t-internalStartTime, task_pd.data);
        cart_task.getAcceleration(t-internalStartTime, task_pdd.data);

        cart_task.getPositionTranslation(t-internalStartTime, task_pTranslation);
		cart_task.getVelocityTranslation(t-internalStartTime, task_pdTranslation);
		cart_task.getAccelerationTranslation(t-internalStartTime, task_pddTranslation);

		cart_task.getPositionOrientation(t-internalStartTime, task_pOrientation);
		cart_task.getVelocityOrientation(t-internalStartTime, task_pdOrientation);
		cart_task.getAccelerationOrientation(t-internalStartTime, task_pddOrientation);

		//log data
//		csv_logger->writeFile(task_p.data);

        // start convert current endeffector pose to eigen
    	curr_ee_pose(0) = cartFrame.p.x();
    	curr_ee_pose(1) = cartFrame.p.y();
    	curr_ee_pose(2) = cartFrame.p.z();
//    	curr_ee_pose(3) = 0;
//    	curr_ee_pose(4) = 0;
//    	curr_ee_pose(5) = 0;
    	curr_ee_pose(3) = cartFrame.M.GetRot().x();
    	curr_ee_pose(4) = cartFrame.M.GetRot().y();
    	curr_ee_pose(5) = cartFrame.M.GetRot().z();

    	curr_ee_vel(0) = velFrame.p.v.x();
    	curr_ee_vel(1) = velFrame.p.v.y();
    	curr_ee_vel(2) = velFrame.p.v.z();
//    	curr_ee_vel(3) = 0;
//    	curr_ee_vel(4) = 0;
//    	curr_ee_vel(5) = 0;
    	curr_ee_vel(3) = velFrame.GetFrame().M.GetRot().x();
    	curr_ee_vel(4) = velFrame.GetFrame().M.GetRot().y();
    	curr_ee_vel(5) = velFrame.GetFrame().M.GetRot().z();

    	curr_ee_poseTranslation(0) = cartFrame.p.x();
		curr_ee_poseTranslation(1) = cartFrame.p.y();
		curr_ee_poseTranslation(2) = cartFrame.p.z();
    	curr_ee_poseOrientation(0) = cartFrame.M.GetRot().x();
		curr_ee_poseOrientation(1) = cartFrame.M.GetRot().y();
		curr_ee_poseOrientation(2) = cartFrame.M.GetRot().z();

		curr_ee_velTranslation(0) = velFrame.p.v.x();
		curr_ee_velTranslation(1) = velFrame.p.v.y();
		curr_ee_velTranslation(2) = velFrame.p.v.z();
		curr_ee_velOrientation(0) = velFrame.GetFrame().M.GetRot().x();
		curr_ee_velOrientation(1) = velFrame.GetFrame().M.GetRot().y();
		curr_ee_velOrientation(2) = velFrame.GetFrame().M.GetRot().z();
    	// stop convert current endeffector pose to eigen

    	// check current configuration for violation of joint limits
		detectedError = false;
    	for (int jointnr = 0; jointnr < DEFAULT_NR_JOINTS; jointnr++) {
    		if (jnt_pos_[jointnr] > jointPosCritic_max[jointnr]){
    			if (jnt_pos_[jointnr] > jointPosLimits_max[jointnr]){
    				l(Error) << "joint " << jointnr << " violates positive joint position limit !!!" << RTT::endlog();
    				detectedError = true;
    			}
    			else{
    				l(Error) << "joint " << jointnr << " is close to positive joint position limit" << RTT::endlog();
    			}
    		}
    		else if (jnt_pos_[jointnr] < jointPosCritic_min[jointnr]){
    			if (jnt_pos_[jointnr] < jointPosLimits_min[jointnr]){
    				l(Error) << "joint " << jointnr << " violates negative joint position limit !!!" << RTT::endlog();
    				detectedError = true;
    			}
    			else{
    				l(Error) << "joint " << jointnr << " is close to negative joint position limit" << RTT::endlog();
    			}
			}

    		if (jnt_vel_[jointnr] > jointVelCritic_max[jointnr]){
    			if (jnt_pos_[jointnr] > jointVelLimits_max[jointnr]){
    				l(Error) << "joint " << jointnr << " violates positive joint velocity limit !!!" << RTT::endlog();
    				detectedError = true;
    			}
    			else{
    				l(Error) << "joint " << jointnr << " is close to positive joint velocity limit" << RTT::endlog();
    			}
    		}
    		else if (jnt_vel_[jointnr] < jointVelCritic_min[jointnr]){
    			if (jnt_pos_[jointnr] > jointVelLimits_min[jointnr]){
    				l(Error) << "joint " << jointnr << " violates negative joint velocity limit !!!" << RTT::endlog();
    				detectedError = true;
    			}
    			else{
    				l(Error) << "joint " << jointnr << " is close to negative joint velocity limit" << RTT::endlog();
    			}
			}
    	}
    	if (detectedError){
    		l(Error) << "task_p.data: " << task_p.data << RTT::endlog();
//    		throw "out";
    	}

//        l(Error) << "pose: " << curr_ee_pose << RTT::endlog();
//        l(Error) << "velo: " << curr_ee_vel << RTT::endlog();
//
//        l(Error) << "cartFrame.M.GetRot().x(): " << cartFrame.M.GetRot().x() / (2*M_PI) * 360 << RTT::endlog();
//        l(Error) << "cartFrame.M.GetRot().y(): " << cartFrame.M.GetRot().y() / (2*M_PI) * 360<< RTT::endlog();
//        l(Error) << "cartFrame.M.GetRot().z(): " << cartFrame.M.GetRot().z() / (2*M_PI) * 360 << RTT::endlog();
//
//        l(Error) << "jnt_pos_: " << jnt_pos_ << RTT::endlog();
//        throw "out";

        // start open loop joint controller (Inverse dynamic controller M+C+G=tau)
//        jnt_trq_cmd_ = M_.data*(task_qdd.data-Kp_joint.asDiagonal()*(jnt_pos_ - task_q.data)-Kd_joint.asDiagonal()*(jnt_vel_-task_qd.data)) + C_.data + G_.data;
//        jnt_trq_cmd_ = M_.data*(task_qdd.data) + C_.data + G_.data;
        // stop open loop joint controller


    	//compute refrence acceleration seperately for translation and orientation
    	ref_accTranslation = task_pddTranslation + Kp_cartTranslation.asDiagonal()*(task_pTranslation - curr_ee_poseTranslation) + Kd_cartTranslation.asDiagonal()*(task_pdTranslation - curr_ee_velTranslation);
    	if (use_euler_orientation){
    		ref_accOrientation = task_pddOrientation + Kp_cartOrientationEuler.asDiagonal()*(task_pOrientation - curr_ee_poseOrientation) + Kd_cartOrientationEuler.asDiagonal()*(task_pdOrientation - curr_ee_velOrientation);
//    		l(Error) << "ref_accOrientation Euler: " << ref_accOrientation << RTT::endlog();
    	}
    	else{
			orientation_helper.Euler2Quaternion(task_pOrientation, desiredCartOrientationQuaternionV, desiredCartOrientationQuaternionU);
			orientation_helper.Euler2Quaternion(curr_ee_poseOrientation, currCartOrientationQuaternionV, currCartOrientationQuaternionU);

			// compute QuaternionProduct
			orientation_helper.QuaternionProduct(desiredCartOrientationQuaternionV, desiredCartOrientationQuaternionU,
													currCartOrientationQuaternionV, currCartOrientationQuaternionU,
													QuaternionProductV, QuaternionProductU);

			orientation_helper.Quaternion2Euler(QuaternionProductV, QuaternionProductU, QuaternionProductEuler);

			ref_accOrientation = Kp_cartOrientationQuaternion.asDiagonal()*2*(QuaternionProductEuler) + Kd_cartOrientationQuaternion.asDiagonal()*(task_pdOrientation - curr_ee_velOrientation); // TODO: task_pdd.data is missing, but here in this case always zero...
//			l(Error) << "ref_accOrientation Quat: " << ref_accOrientation << RTT::endlog();
    	}
//		l(Error) << "ref_accTranslation: " << ref_accTranslation << RTT::endlog();
//    	l(Error) << "ref_accOrientation: " << ref_accOrientation << RTT::endlog();
		ref_acc(0) = ref_accTranslation(0);
		ref_acc(1) = ref_accTranslation(1);
		ref_acc(2) = ref_accTranslation(2);
		ref_acc(3) = ref_accOrientation(0);
		ref_acc(4) = ref_accOrientation(1);
		ref_acc(5) = ref_accOrientation(2);

		if (use_original_khatib_controller){
			//Start Khatib endeffector motion controller
			h = C_.data + G_.data;
			Lamda = (jac_.data * M_.data.inverse() * jac_.data.transpose() + tmpeye66).inverse(); //add regression for better inverse computation
			CG_bar = Lamda*(jac_.data * M_.data.inverse() * (h) - jac_dot_.data * jnt_vel_);
			Forces  = Lamda * ref_acc + CG_bar;
			jnt_trq_cmd_Motion_Khatib = jac_.data.transpose()*Forces;
			//Stop Khatib endeffector motion controller


			//Start Khatib nullspace controller
			tau_0 = Kp_joint.asDiagonal()*(q_des_Nullspace.data - jnt_pos_) - Kd_joint.asDiagonal()*(jnt_vel_);
			N = identity77 - jac_.data.transpose() * ( Lamda * jac_.data * M_.data );
	//        N = identity77 - jac_.data.transpose() * ( jac_.data );
			jnt_trq_cmd_Nullspace_Khatib = N * tau_0;
			//Stop Khatib nullspace controller

			jnt_trq_cmd_ = jnt_trq_cmd_Motion_Khatib + 0.1 * jnt_trq_cmd_Nullspace_Khatib;
		}
		else{
		    //compute constrained jacobian //TODO: check...
		    jac_cstr_ = jac_.data;
		    jac_cstr_.row(0).setZero();
		    jac_cstr_.row(1).setZero();
		    //jac_cstr_.row(2).setZero();
		    jac_cstr_.row(3).setZero();
		    jac_cstr_.row(4).setZero();
		    jac_cstr_.row(5).setZero();

		    //compute constrained projection variables
			jac_cstr_MPI = (jac_cstr_.transpose() * jac_cstr_ + tmpeye77).inverse() * jac_cstr_.transpose();
			P = identity77 - (jac_cstr_MPI * jac_cstr_);
			M_cstr_ = P * M_.data +  identity77 - P;
			C_cstr_ = -(jac_cstr_MPI * jac_cstr_);
			Lamda_cstr = (jac_.data * M_cstr_.inverse() * P * jac_.data.transpose() + tmpeye66).inverse();

			//Start Khatib projected endeffector motion controller
			h = C_.data + G_.data;
			Forces_cstr = Lamda_cstr * ref_acc + Lamda_cstr * (jac_.data * M_cstr_.inverse() * P * h - (jac_dot_.data + jac_.data * M_cstr_.inverse() * C_cstr_)*jnt_vel_ );
			jnt_trq_cmd_Motion_Projected = P * jac_.data.transpose()*Forces_cstr;
			//Stop Khatib projected endeffector motion controller


			//Start constrained nullspace controller
			tau_0 = Kp_joint.asDiagonal()*(q_des_Nullspace.data - jnt_pos_) - Kd_joint.asDiagonal()*(jnt_vel_);
			N = identity77 - jac_.data.transpose() * ((jac_.data * M_cstr_.inverse() * P * jac_.data.transpose()).inverse() * jac_.data * M_cstr_.inverse() * P);
			jnt_trq_cmd_Nullspace_Projected = P * N * tau_0;
			//Stop constrained nullspace controller

	        //Start external forces controller

                jnt_trq_cmd_Force_Projected = (identity77 - P) * (h) + jac_cstr_.transpose() * lambda_des + (identity77 - P) * M_.data * M_cstr_.inverse() * (P * M_.data * currJntAcc + C_cstr_ * jnt_vel_ );

	        //Stop external forces controller

            jnt_trq_cmd_ = jnt_trq_cmd_Motion_Projected + 0.1 * jnt_trq_cmd_Nullspace_Projected + jnt_trq_cmd_Force_Projected;
		}

//        l(Error) << "jac_cstr_: " << jac_cstr_ << RTT::endlog();
//        l(Error) << "jac_cstr_MPI: " << jac_cstr_MPI << RTT::endlog();
//		l(Error) << "tau_0: " << tau_0 << RTT::endlog();
//		l(Error) << "jnt_trq_cmd_Motion_Khatib: " << jnt_trq_cmd_Motion_Khatib << RTT::endlog();
//		l(Error) << "jnt_trq_cmd_Motion_Projected: " << jnt_trq_cmd_Motion_Projected << RTT::endlog();
//
//		l(Error) << "jnt_trq_cmd_Nullspace_Khatib: " << jnt_trq_cmd_Nullspace_Khatib << RTT::endlog();
//		l(Error) << "jnt_trq_cmd_Nullspace_Projected: " << jnt_trq_cmd_Nullspace_Projected << RTT::endlog();

//		if (getSimulationTime()-internalStartTime > start_time+3) {
//			throw "out";
//		}

	} else {
		// start gravitation compensation controller
//        kg_.setConstant(1); // heavily overshooting!!!! BEWARE
//        jnt_trq_cmd_ = kg_.asDiagonal() * G_.data;
		// stop gravitation compensation controller


		// start simple joint position controller
        kg_.setConstant(1);
        jnt_trq_cmd_ = G_.data + kg_.asDiagonal() * (q_des_FirstPoint.data - jnt_pos_);
        // stop simple joint position controller

        // start simple cartesian space controller by Niels (=> needs further tuning of gains)
//		cart_task.getPositionTranslation(start_time, task_pTranslation);
//		cart_task.getVelocityTranslation(start_time, task_pdTranslation);
//		cart_task.getPositionOrientation(start_time, task_pOrientation);
//		cart_task.getVelocityOrientation(start_time, task_pdOrientation);
//
//		ref_accTranslation = 100.0*diagonal33*(task_pTranslation - curr_ee_poseTranslation) + 20.0*diagonal33*(task_pdTranslation - curr_ee_velTranslation);
//		ref_accOrientation =   2.0*diagonal33*(task_pOrientation - curr_ee_poseOrientation) +  0.0*diagonal33*(task_pdOrientation - curr_ee_velOrientation);
//		l(Error) << "deviation Translation: " << task_pTranslation - curr_ee_poseTranslation << RTT::endlog();
//		l(Error) << "deviation Orientation: " << task_pOrientation - curr_ee_poseOrientation << RTT::endlog();
//		l(Error) << "ref_accTranslation: " << ref_accTranslation << RTT::endlog();
//		l(Error) << "ref_accOrientation: " << ref_accOrientation << RTT::endlog();
//		Forces(0) = ref_accTranslation(0);
//		Forces(1) = ref_accTranslation(1);
//		Forces(2) = ref_accTranslation(2);
//		Forces(3) = ref_accOrientation(0);
//		Forces(4) = ref_accOrientation(1);
//		Forces(5) = ref_accOrientation(2);
//		jnt_trq_cmd_ = jac_.data.transpose()*Forces;
//		l(Error) << "jnt_trq_cmd_: " << jnt_trq_cmd_ << RTT::endlog();
		// stop simple cartesian space controller by Niels
	}

//    RTT::log(RTT::Error) << "jnt_trq_cmd_ :\n" << jnt_trq_cmd_ << RTT::endlog();
//    throw "out";



    // DON'T TOUCH BELOW
	// convert eigen back to rci
	for (int i = 0; i < outJntTrq->getDimension(); i++) {
		outJntTrq->setFromNm(i, jnt_trq_cmd_.data()[i]);
	}

	// write torques to robot
	if (cmdJntTrq_Port.connected()) {
		cmdJntTrq_Port.write(outJntTrq);
	}

	last_SimulationTime = getSimulationTime();
	lastJntVel = jnt_vel_;
}

void RttLwrOSFController::stopHook() {
	csv_logger->closeFile();
	l(Info) << "executes stopping !" << endlog();
}

void RttLwrOSFController::cleanupHook() {
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
//ORO_LIST_COMPONENT_TYPE(RttLwrOSFController)
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(RttLwrOSFController)
