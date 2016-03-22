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
bool use_original_khatib_controller = true;

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

	l(Info) << "constructed !" << endlog();
}

bool RttLwrOSFController::configureHook() {

	initKDLTools();


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
    q_des_FirstPoint.data(3) =-1.5562;
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
    ref_accOrientationEuler.resize(3);
    //stop variables for quaternion feedback stuff

    lambda_des.resize(6);
    lambda_des.setConstant(0.0);
	lambda_des[0] = -5; //desired endeffector force

    rne_torques.resize(DEFAULT_NR_JOINTS);
    ext_force.resize(DEFAULT_NR_JOINTS);

    Kp_joint.resize(DEFAULT_NR_JOINTS);
    Kd_joint.resize(DEFAULT_NR_JOINTS);
//    Kp_joint.setConstant(2.0);
//	Kd_joint.setConstant(1.4);
    Kp_joint.setConstant(20.0);
	Kd_joint.setConstant(6.0);

    Kp_cartTranslation.resize(3);
    Kd_cartTranslation.resize(3);

    Kp_cartTranslation.setConstant(50.0);
    Kd_cartTranslation.setConstant(14.0);

    Kp_cartOrientation.resize(3);
    Kd_cartOrientation.resize(3);

    Kp_cartOrientation.setConstant(2500.0);
	Kd_cartOrientation.setConstant(100.0);


    tau_0.resize(DEFAULT_NR_JOINTS);

    //Khatib controller:
	Lamda.resize(6,6);
	Lamda_cstr.resize(6,6);
	CG_bar.resize(6);
	Forces.resize(6);
	Forces_cstr.resize(6);

	lastJntVel = rci::JointVelocities::create(DEFAULT_NR_JOINTS, 0.0);

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
	currJntAcc = rci::JointAccelerations::fromRad_ss( (lastJntVel->rad_sVector() - currJntVel->rad_sVector()) * (1 / delta_t) );

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

        cart_task.getPositionTranslation(t-internalStartTime, task_pTranslation.data);
		cart_task.getVelocityTranslation(t-internalStartTime, task_pdTranslation.data);
		cart_task.getAccelerationTranslation(t-internalStartTime, task_pddTranslation.data);

		cart_task.getPositionOrientation(t-internalStartTime, task_pOrientation.data);
		cart_task.getVelocityOrientation(t-internalStartTime, task_pdOrientation.data);
		cart_task.getAccelerationOrientation(t-internalStartTime, task_pddOrientation.data);

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
    	for (int jointnr = 0; jointnr < DEFAULT_NR_JOINTS; jointnr++) {
    		if (jnt_pos_[jointnr] > jointPosCritic_max[jointnr]){
    			if (jnt_pos_[jointnr] > jointPosLimits_max[jointnr]){
    				l(Error) << "joint " << jointnr << " violates positive joint position limit !!!" << RTT::endlog();
    				throw "out";
    			}
    			else{
    				l(Error) << "joint " << jointnr << " is close to positive joint position limit" << RTT::endlog();
    			}
    		}
    		else if (jnt_pos_[jointnr] < jointPosCritic_min[jointnr]){
    			if (jnt_pos_[jointnr] < jointPosLimits_min[jointnr]){
    				l(Error) << "joint " << jointnr << " violates negative joint position limit !!!" << RTT::endlog();
    				throw "out";
    			}
    			else{
    				l(Error) << "joint " << jointnr << " is close to negative joint position limit" << RTT::endlog();
    			}
			}

    		if (jnt_vel_[jointnr] > jointVelCritic_max[jointnr]){
    			if (jnt_pos_[jointnr] > jointVelLimits_max[jointnr]){
    				l(Error) << "joint " << jointnr << " violates positive joint velocity limit !!!" << RTT::endlog();
    				throw "out";
    			}
    			else{
    				l(Error) << "joint " << jointnr << " is close to positive joint velocity limit" << RTT::endlog();
    			}
    		}
    		else if (jnt_vel_[jointnr] < jointVelCritic_min[jointnr]){
    			if (jnt_pos_[jointnr] > jointVelLimits_min[jointnr]){
    				l(Error) << "joint " << jointnr << " violates negative joint velocity limit !!!" << RTT::endlog();
    				throw "out";
    			}
    			else{
    				l(Error) << "joint " << jointnr << " is close to negative joint velocity limit" << RTT::endlog();
    			}
			}
    	}

//        l(Error) << "pose: " << curr_ee_pose << RTT::endlog();
//        l(Error) << "velo: " << curr_ee_vel << RTT::endlog();
//
//        l(Error) << "cartFrame.M.GetRot().x(): " << cartFrame.M.GetRot().x() / (2*M_PI) * 360 << RTT::endlog();
//        l(Error) << "cartFrame.M.GetRot().y(): " << cartFrame.M.GetRot().y() / (2*M_PI) * 360<< RTT::endlog();
//        l(Error) << "cartFrame.M.GetRot().z(): " << cartFrame.M.GetRot().z() / (2*M_PI) * 360 << RTT::endlog();
//

//        l(Error) << "jnt_pos_: " << jnt_pos_ << RTT::endlog();
//    	l(Error) << "task_p.data: " << task_p.data << RTT::endlog();
//
//        throw "out";

        // start open loop joint controller (Inverse dynamic controller M+C+G=tau)
//        jnt_trq_cmd_ = M_.data*(task_qdd.data-Kp_joint.asDiagonal()*(jnt_pos_ - task_q.data)-Kd_joint.asDiagonal()*(jnt_vel_-task_qd.data)) + C_.data + G_.data;
//        jnt_trq_cmd_ = M_.data*(task_qdd.data) + C_.data + G_.data;
        // stop open loop joint controller

    	//start quaternion feedback stuff
//    	desiredCartOrientation = rci::Orientation::fromEulerAngles( task_p.data(3), task_p.data(4), task_p.data(5) );
//    	currCartOrientation = rci::Orientation::fromEulerAngles( curr_ee_pose(3), curr_ee_pose(4), curr_ee_pose(5) );
//
//    	//get Quaternion representation
//    	desiredCartOrientationQuaternionV    = desiredCartOrientation->q0();
//    	desiredCartOrientationQuaternionU(0) = desiredCartOrientation->q1();
//    	desiredCartOrientationQuaternionU(1) = desiredCartOrientation->q2();
//    	desiredCartOrientationQuaternionU(2) = desiredCartOrientation->q3();
//
//    	currCartOrientationQuaternionV    = currCartOrientation->q0();
//    	currCartOrientationQuaternionU(0) = currCartOrientation->q1();
//    	currCartOrientationQuaternionU(1) = currCartOrientation->q2();
//    	currCartOrientationQuaternionU(2) = currCartOrientation->q3();
//
//    	// compute QuaternionProduct
//    	QuaternionProductV = 	desiredCartOrientationQuaternionV * currCartOrientationQuaternionV -
//    							desiredCartOrientationQuaternionU.transpose() * currCartOrientationQuaternionU;
//		QuaternionProductU = 	desiredCartOrientationQuaternionV * currCartOrientationQuaternionU +
//								currCartOrientationQuaternionV * desiredCartOrientationQuaternionU +
//								desiredCartOrientationQuaternionU.cross(currCartOrientationQuaternionU);
//
//		// compute log(Quaternion)
//		if(QuaternionProductU(0) == 0.0 && QuaternionProductU(1) == 0.0 && QuaternionProductU(2) == 0.0){
//			QuaternionProductEuler.setZero();
//		}
//		else{
//			QuaternionProductEuler = acos(QuaternionProductV) * QuaternionProductU / QuaternionProductU.norm(); //or use .squaredNorm() here??
//		}
//
//		ref_accOrientationEuler = Kp_cartOrientation.asDiagonal()*2*(QuaternionProductEuler) - Kd_cartOrientation.asDiagonal()*(curr_ee_velOrientation); // TODO: task_pdd.data is missing
		//stop quaternion feedback stuff


		//compute refrence acceleration
		ref_accTranslation = task_pddTranslation.data + Kd_cartTranslation.asDiagonal()*(task_pdTranslation.data - curr_ee_velTranslation) + Kp_cartTranslation.asDiagonal()*(task_pTranslation.data - curr_ee_poseTranslation);
		ref_accOrientation = task_pddOrientation.data + Kd_cartOrientation.asDiagonal()*(task_pdOrientation.data - curr_ee_velOrientation) + Kp_cartOrientation.asDiagonal()*(task_pOrientation.data - curr_ee_poseOrientation);
//		l(Error) << "ref_accTranslation: " << ref_accTranslation << RTT::endlog();
//		l(Error) << "ref_accOrientation: " << ref_accOrientation << RTT::endlog();
		ref_acc(0) = ref_accTranslation(0);
		ref_acc(1) = ref_accTranslation(1);
		ref_acc(2) = ref_accTranslation(2);
		ref_acc(3) = ref_accOrientation(0);
		ref_acc(4) = ref_accOrientation(1);
		ref_acc(5) = ref_accOrientation(2);

//    	ref_acc(3) = ref_accOrientationEuler(0);
//    	ref_acc(4) = ref_accOrientationEuler(1);
//    	ref_acc(5) = ref_accOrientationEuler(2);

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
		    jac_cstr_.row(2).setZero();
		//    jac_cstr_.row(0).setZero();
		//    jac_cstr_.row(1).setZero();
		//    jac_cstr_.row(3).setZero();
		//    jac_cstr_.row(4).setZero();
		//    jac_cstr_.row(5).setZero();

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
	        //jnt_trq_cmd_Force_Projected = (identity77 - P) * (h) + (identity77 - P) * M_.data * M_cstr_.inverse() * (P * M_.data * currJntAcc * + C_cstr_) + jac_cstr_.transpose() * lambda_des;
	        //Stop external forces controller

			jnt_trq_cmd_ = jnt_trq_cmd_Motion_Projected;// + 0.1 * jnt_trq_cmd_Nullspace_Projected; // + jnt_trq_cmd_Force_Projected
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
//		cart_task.getPositionTranslation(start_time, task_pTranslation.data);
//		cart_task.getVelocityTranslation(start_time, task_pdTranslation.data);
//		cart_task.getPositionOrientation(start_time, task_pOrientation.data);
//		cart_task.getVelocityOrientation(start_time, task_pdOrientation.data);
//
//		ref_accTranslation = 100.0*diagonal33*(task_pTranslation.data - curr_ee_poseTranslation) + 20.0*diagonal33*(task_pdTranslation.data - curr_ee_velTranslation);
//		ref_accOrientation =   2.0*diagonal33*(task_pOrientation.data - curr_ee_poseOrientation) +  0.0*diagonal33*(task_pdOrientation.data - curr_ee_velOrientation);
//		l(Error) << "deviation Translation: " << task_pTranslation.data - curr_ee_poseTranslation << RTT::endlog();
//		l(Error) << "deviation Orientation: " << task_pOrientation.data - curr_ee_poseOrientation << RTT::endlog();
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
	lastJntVel = currJntVel;
}

void RttLwrOSFController::stopHook() {
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
