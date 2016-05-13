#include <rtt/Component.hpp>


#include "rtt-khatib.hpp"


RTTKhatibForceController::RTTKhatibForceController(std::string const& name, unsigned int numJoints) :
		RTT::TaskContext(name),
		currTaskEEPosTranslation_Flow(RTT::NoData),
		currTaskEEVelTranslation_Flow(RTT::NoData),
//		currTaskEEAccTranslation_Flow(RTT::NoData),
		refTaskEEPosTranslation_Flow(RTT::NoData),
		refTaskEEVelTranslation_Flow(RTT::NoData),
		refTaskEEAccTranslation_Flow(RTT::NoData),
		currTaskEEPosOrientation_Flow(RTT::NoData),
		currTaskEEVelOrientation_Flow(RTT::NoData),
//		currTaskEEAccOrientation_Flow(RTT::NoData),
		refTaskEEPosOrientation_Flow(RTT::NoData),
		refTaskEEVelOrientation_Flow(RTT::NoData),
		refTaskEEAccOrientation_Flow(RTT::NoData),
		numJoints(numJoints)
		{

	//output joint torque port
    addPort("cmdJntTrq", cmdJntTrq_Port);
    cmdJntTrq = rci::JointTorques::create(numJoints, 0.0);
    cmdJntTrq_Port.setDataSample(cmdJntTrq);

    //input joint ports
    this->addPort("currJntPos", currJntPos_Port);
    this->addPort("currJntVel", currJntVel_Port);

    //input cartesian translation ports
    addPort("currTaskEEPosTranslation", currTaskEEPosTranslation_Port);
    addPort("currTaskEEVelTranslation", currTaskEEVelTranslation_Port);
    addPort("refTaskEEPosTranslation", refTaskEEPosTranslation_Port);
    addPort("refTaskEEVelTranslation", refTaskEEVelTranslation_Port);
    addPort("refTaskEEAccTranslation", refTaskEEAccTranslation_Port);

    //input cartesian orientation ports
    addPort("currTaskEEPosOrientation", currTaskEEPosOrientation_Port);
    addPort("currTaskEEVelOrientation", currTaskEEVelOrientation_Port);
    addPort("refTaskEEPosOrientation", refTaskEEPosOrientation_Port);
    addPort("refTaskEEVelOrientation", refTaskEEVelOrientation_Port);
    addPort("refTaskEEAccOrientation", refTaskEEAccOrientation_Port);

//    l(Info) << "constructed!" << endlog();
}

bool RTTKhatibForceController::configureHook() {
    if (!(cmdJntTrq_Port.connected() &&
    		currTaskEEPosTranslation_Port.connected() && currTaskEEVelTranslation_Port.connected() &&
    		refTaskEEPosTranslation_Port.connected() && refTaskEEVelTranslation_Port.connected() && refTaskEEAccTranslation_Port.connected() &&
    		currTaskEEPosOrientation_Port.connected() && currTaskEEVelOrientation_Port.connected() &&
    		refTaskEEPosOrientation_Port.connected() && refTaskEEVelOrientation_Port.connected() && refTaskEEAccOrientation_Port.connected()
    		)) {
        return false;
    }

//    l(Info) << "configured!" << endlog();
    return true;
}

bool RTTKhatibForceController::startHook() {

	// read parameters from config-file
	addProperty( "use_original_khatib_controller", use_original_khatib_controller ).doc("use_original_khatib_controller Example Description");
	addProperty( "use_euler_orientation", use_euler_orientation ).doc("use_euler_orientation Example Description");

	addProperty( "Kp_cartTranslationKhatibGain", Kp_cartTranslationKhatibGain ).doc("KhatibGain1 Example Description");
	addProperty( "Kd_cartTranslationKhatibGain", Kd_cartTranslationKhatibGain ).doc("KhatibGain2 Example Description");
	addProperty( "Kp_cartOrientationEulerKhatibGain", Kp_cartOrientationEulerKhatibGain ).doc("EulerKhatibGain3 Example Description");
	addProperty( "Kd_cartOrientationEulerKhatibGain", Kd_cartOrientationEulerKhatibGain ).doc("EulerKhatibGain4 Example Description");
	addProperty( "Kp_cartOrientationQuaternionKhatibGain", Kp_cartOrientationQuaternionKhatibGain ).doc("QuaternionKhatibGain3 Example Description");
	addProperty( "Kd_cartOrientationQuaternionKhatibGain", Kd_cartOrientationQuaternionKhatibGain ).doc("QuaternionKhatibGain4 Example Description");
	addProperty( "Kp_jointKhatibGain", Kp_jointKhatibGain ).doc("KhatibGain5 Example Description");
	addProperty( "Kd_jointKhatibGain", Kd_jointKhatibGain ).doc("KhatibGain6 Example Description");

	addProperty( "Kp_cartTranslationConstrainedGain", Kp_cartTranslationConstrainedGain ).doc("ConstrainedGain1 Example Description");
	addProperty( "Kd_cartTranslationConstrainedGain", Kd_cartTranslationConstrainedGain ).doc("ConstrainedGain2 Example Description");
	addProperty( "Kp_cartOrientationEulerConstrainedGain", Kp_cartOrientationEulerConstrainedGain ).doc("EulerConstrainedGain3 Example Description");
	addProperty( "Kd_cartOrientationEulerConstrainedGain", Kd_cartOrientationEulerConstrainedGain ).doc("EulerConstrainedGain4 Example Description");
	addProperty( "Kp_cartOrientationQuaternionConstrainedGain", Kp_cartOrientationQuaternionConstrainedGain ).doc("QuaternionConstrainedGain3 Example Description");
	addProperty( "Kd_cartOrientationQuaternionConstrainedGain", Kd_cartOrientationQuaternionConstrainedGain ).doc("QuaternionConstrainedGain4 Example Description");
	addProperty( "Kp_jointConstrainedGain", Kp_jointConstrainedGain ).doc("ConstrainedGain5 Example Description");
	addProperty( "Kd_jointConstrainedGain", Kd_jointConstrainedGain ).doc("ConstrainedGain6 Example Description");

	//fill gain matrices
    Kp_cartTranslation.resize(3);
    Kd_cartTranslation.resize(3);
    Kp_cartOrientationEuler.resize(3);
    Kd_cartOrientationEuler.resize(3);
    Kp_cartOrientationQuaternion.resize(3);
	Kd_cartOrientationQuaternion.resize(3);
    Kp_joint.resize(numJoints);
    Kd_joint.resize(numJoints);

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

    // set helper matrices
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

	//
	ref_accTranslation.resize(3);
	ref_accOrientation.resize(3);
	ref_acc.resize(6);
	Lamda.resize(6,6);
	Lamda_cstr.resize(6,6);
	CG_bar.resize(6);
	h.resize(6, 1);
	Forces.resize(6);
	Forces_cstr.resize(6);
	tau_0.resize(numJoints);
	N.resize(numJoints,numJoints);
	P.resize(numJoints,numJoints);
	jac_cstr_.resize(6, numJoints);
	jac_cstr_MPI.resize(numJoints, 6);
	M_cstr_.resize(numJoints,numJoints);
	C_cstr_.resize(numJoints,numJoints);

    lambda_des.resize(6);
    lambda_des.setConstant(0.0);
	lambda_des[2] = -5; //desired endeffector force, use z-axis here

	jnt_trq_cmd_Motion_Khatib.resize(numJoints);
	jnt_trq_cmd_Nullspace_Khatib.resize(numJoints);
	jnt_trq_cmd_Motion_Projected.resize(numJoints);
	jnt_trq_cmd_Nullspace_Projected.resize(numJoints);
	jnt_trq_cmd_Force_Projected.resize(numJoints);

	M_.resize(numJoints);
	C_.resize(numJoints);
	G_.resize(numJoints);
	jac_.resize(numJoints);
	jac_.data.setZero();
	jac_dot_.resize(numJoints);
	jac_dot_.data.setZero();

    q_des_Nullspace.resize(numJoints); // points endeffector to center of circle
    q_des_Nullspace.data.setZero();
    q_des_FirstPoint.resize(numJoints); // points endeffector to first point of circle
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

//    //start variables for quaternion feedback stuff
//	desiredCartOrientationQuaternionU.resize(3);
//	currCartOrientationQuaternionU.resize(3);
//	QuaternionProductU.resize(3);
//    QuaternionProductEuler.resize(3);
//    //stop variables for quaternion feedback stuff

//	l(Info) << "started!" << endlog();
    return true;
}

void RTTKhatibForceController::updateHook() {

    //read input joint ports
    currJntPos_Flow = currJntPos_Port.read(currJntPos);
    currJntVel_Flow = currJntVel_Port.read(currJntVel);

	//read input cartesian translation ports
	currTaskEEPosTranslation_Flow = currTaskEEPosTranslation_Port.read(currTaskEEPosTranslation);
	currTaskEEVelTranslation_Flow = currTaskEEVelTranslation_Port.read(currTaskEEVelTranslation);
	refTaskEEPosTranslation_Flow = refTaskEEPosTranslation_Port.read(refTaskEEPosTranslation);
	refTaskEEVelTranslation_Flow = refTaskEEVelTranslation_Port.read(refTaskEEVelTranslation);
	refTaskEEAccTranslation_Flow = refTaskEEAccTranslation_Port.read(refTaskEEAccTranslation);

	//read input cartesian orientation ports
	currTaskEEPosOrientation_Flow = currTaskEEPosOrientation_Port.read(currTaskEEPosOrientation);
	currTaskEEVelOrientation_Flow = currTaskEEVelOrientation_Port.read(currTaskEEVelOrientation);
	refTaskEEPosOrientation_Flow = refTaskEEPosOrientation_Port.read(refTaskEEPosOrientation);
	refTaskEEVelOrientation_Flow = refTaskEEVelOrientation_Port.read(refTaskEEVelOrientation);
	refTaskEEAccOrientation_Flow = refTaskEEAccOrientation_Port.read(refTaskEEAccOrientation);


//	// check for NoData
//	if (currTaskEEPosTranslation_Flow == RTT::NoData) {
//		// skip this step, because we don't receive all the necessary data.
//		return;
//	}
//	...
//    if (currTaskEEPosTranslation_Flow == RTT::NewData) {
//
//    }
//	...



	//compute refrence acceleration seperately for translation and orientation
	ref_accTranslation = refTaskEEAccTranslation + Kp_cartTranslation.asDiagonal()*(refTaskEEPosTranslation - currTaskEEPosTranslation) + Kd_cartTranslation.asDiagonal()*(refTaskEEVelTranslation - currTaskEEVelTranslation);
//	if (use_euler_orientation){
		ref_accOrientation = refTaskEEAccOrientation + Kp_cartOrientationEuler.asDiagonal()*(refTaskEEPosOrientation - currTaskEEPosOrientation) + Kd_cartOrientationEuler.asDiagonal()*(refTaskEEVelOrientation - currTaskEEVelOrientation);
//	}
//	else{
//		orientation_helper.Euler2Quaternion(refTaskEEPosOrientation, desiredCartOrientationQuaternionV, desiredCartOrientationQuaternionU);
//		orientation_helper.Euler2Quaternion(currTaskEEPosOrientation, currCartOrientationQuaternionV, currCartOrientationQuaternionU);
//
//		// compute QuaternionProduct
//		orientation_helper.QuaternionProduct(desiredCartOrientationQuaternionV, desiredCartOrientationQuaternionU,
//												currCartOrientationQuaternionV, currCartOrientationQuaternionU,
//												QuaternionProductV, QuaternionProductU);
//
//		orientation_helper.Quaternion2Euler(QuaternionProductV, QuaternionProductU, QuaternionProductEuler);
//
//		ref_accOrientation = Kp_cartOrientationQuaternion.asDiagonal()*2*(QuaternionProductEuler) + Kd_cartOrientationQuaternion.asDiagonal()*(refTaskEEVelOrientation - currTaskEEVelOrientation); // TODO: task_pdd.data is missing, but here in this case always zero...
//	}

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
		CG_bar = Lamda*(jac_.data * M_.data.inverse() * (h) - jac_dot_.data * currJntVel);
		Forces  = Lamda * ref_acc + CG_bar;
		jnt_trq_cmd_Motion_Khatib = jac_.data.transpose()*Forces;
		//Stop Khatib endeffector motion controller

		//Start Khatib nullspace controller
		tau_0 = Kp_joint.asDiagonal()*(q_des_Nullspace.data - currJntPos) - Kd_joint.asDiagonal()*(currJntVel);
		N = identity77 - jac_.data.transpose() * ( Lamda * jac_.data * M_.data );
        //N = identity77 - jac_.data.transpose() * ( jac_.data );
		jnt_trq_cmd_Nullspace_Khatib = N * tau_0;
		//Stop Khatib nullspace controller

		cmdJntTrq = jnt_trq_cmd_Motion_Khatib + 0.1 * jnt_trq_cmd_Nullspace_Khatib;
	}
	else{
	    //Start compute constrained jacobian
	    jac_cstr_ = jac_.data;
	    jac_cstr_.row(0).setZero();
	    jac_cstr_.row(1).setZero();
	    //jac_cstr_.row(2).setZero();
	    jac_cstr_.row(3).setZero();
	    jac_cstr_.row(4).setZero();
	    jac_cstr_.row(5).setZero();
	    //Stop compute constrained jacobian

	    //Start compute constrained projection variables
		jac_cstr_MPI = (jac_cstr_.transpose() * jac_cstr_ + tmpeye77).inverse() * jac_cstr_.transpose();
		P = identity77 - (jac_cstr_MPI * jac_cstr_);
		M_cstr_ = P * M_.data +  identity77 - P;
		C_cstr_ = -(jac_cstr_MPI * jac_cstr_);
		Lamda_cstr = (jac_.data * M_cstr_.inverse() * P * jac_.data.transpose() + tmpeye66).inverse();
		//Stop compute constrained projection variables

		//Start Khatib projected endeffector motion controller
		h = C_.data + G_.data;
		Forces_cstr = Lamda_cstr * ref_acc + Lamda_cstr * (jac_.data * M_cstr_.inverse() * P * h - (jac_dot_.data + jac_.data * M_cstr_.inverse() * C_cstr_)*currJntVel );
		jnt_trq_cmd_Motion_Projected = P * jac_.data.transpose()*Forces_cstr;
		//Stop Khatib projected endeffector motion controller

		//Start constrained nullspace controller
		tau_0 = Kp_joint.asDiagonal()*(q_des_Nullspace.data - currJntPos) - Kd_joint.asDiagonal()*(currJntVel);
		N = identity77 - jac_.data.transpose() * ((jac_.data * M_cstr_.inverse() * P * jac_.data.transpose()).inverse() * jac_.data * M_cstr_.inverse() * P);
		jnt_trq_cmd_Nullspace_Projected = P * N * tau_0;
		//Stop constrained nullspace controller

        //Start external forces controller
		jnt_trq_cmd_Force_Projected = (identity77 - P) * (h) + jac_cstr_.transpose() * lambda_des + (identity77 - P) * M_.data * M_cstr_.inverse() * (P * M_.data * currJntAcc + C_cstr_ * currJntVel );
        //Stop external forces controller

		cmdJntTrq = jnt_trq_cmd_Motion_Projected + 0.1 * jnt_trq_cmd_Nullspace_Projected + jnt_trq_cmd_Force_Projected;
	}


    if (cmdJntTrq_Port.connected()) {
        cmdJntTrq_Port.write(cmdJntTrq);
    }
}

void RTTKhatibForceController::stopHook() {
//	l(Info) << "stopped!" << endlog();
}

void RTTKhatibForceController::cleanupHook() {
//	l(Info) << "cleaned!" << endlog();
}


ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(RTTKhatibForceController)
