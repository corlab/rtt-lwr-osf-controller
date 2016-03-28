#include <rtt/Component.hpp>
#include <iostream>
#include <nemo/Vector.h>
#include <nemo/Mapping.h>
#include <rtt/Activity.hpp>
#include "rtt-motion-primitive-wrapper.hpp"

#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace rci;

#define l(lvl) log(lvl) << "[" << this->getName() << "] "


RttMotionPrimitiveWrapper::RttMotionPrimitiveWrapper(std::string const& name) :
		RttMotionPrimitiveWrapper(name, "lwr_arm_base_link", "lwr_arm_7_link", 7),
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

	this->addOperation("parseURDFforKDL", &RttMotionPrimitiveWrapper::parseURDFforKDL,
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

bool RttMotionPrimitiveWrapper::configureHook() {

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
    q_des_FirstPoint.data(0) = 0.0974;
    q_des_FirstPoint.data(1) = 0.0534;
    q_des_FirstPoint.data(2) = 0.0872;
    q_des_FirstPoint.data(3) =-1.7562;
    q_des_FirstPoint.data(4) =-0.0325;
    q_des_FirstPoint.data(5) = 1.2971;
    q_des_FirstPoint.data(6) = 0.0;

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

	l(Info) << "configured !" << endlog();
	return true;
}

bool RttMotionPrimitiveWrapper::startHook() {
	l(Info) << "started !" << endlog();
	return true;

}

double RttMotionPrimitiveWrapper::getSimulationTime(){
    return 1E-9
            * RTT::os::TimeService::ticks2nsecs(
                    RTT::os::TimeService::Instance()->getTicks());
}


void RttMotionPrimitiveWrapper::updateHook() {


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

void RttMotionPrimitiveWrapper::stopHook() {
	l(Info) << "executes stopping !" << endlog();
}

void RttMotionPrimitiveWrapper::cleanupHook() {
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
ORO_LIST_COMPONENT_TYPE(RttMotionPrimitiveWrapper)
