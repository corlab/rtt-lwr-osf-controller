/* Author: Milad Malekzadeh
 * Date:   16 August 2016
 *
 * Description:
 */

#include "PositionController.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

PositionController::PositionController(std::string const & name) :
		RTT::TaskContext(name), receiveTranslationOnly(true) {
	//prepare operations
	addOperation("setDOFsize", &PositionController::setDOFsize, this).doc(
			"set DOF size");
	addOperation("setGains", &PositionController::setGains, this).doc(
			"set gains");
	addOperation("setGainsOrientation",
			&PositionController::setGainsOrientation, this).doc(
			"set gains orientation");
	addOperation("displayStatus", &PositionController::displayStatus, this).doc(
			"print status");

	addOperation("preparePorts", &PositionController::preparePorts, this).doc(
			"preparePorts");

	addOperation("setTranslationOnly", &PositionController::setTranslationOnly,
			this, RTT::ClientThread).doc(
			"set translation only, or use also orientation");

	//other stuff
	gainP = 100;
	gainD = 20;
	portsArePrepared = false;
	setTranslationOnly(true);

	rotx = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX());
	roty = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());
	rotz = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
    euler_diff_pos.setZero();
	euler_diff_vel.setZero();
//	rotx.axis() = Eigen::Vector3f::UnitX();
//	roty.axis() = Eigen::Vector3f::UnitY();
//	rotz.axis() = Eigen::Vector3f::UnitZ();
}

bool PositionController::configureHook() {
	return true;
}

bool PositionController::startHook() {
	if (!in_robotstatus_port.connected()) {
		RTT::log(RTT::Info) << "in_robotstatus_port not connected"
				<< RTT::endlog();
		return false;
	}
	if (!in_desiredTaskSpacePosition_port.connected()) {
		RTT::log(RTT::Info) << "in_desiredTaskSpacePosition_port not connected"
				<< RTT::endlog();
		return false;
	}

	//TODO add other ports here

	if (!out_torques_port.connected()) {
		RTT::log(RTT::Info) << "out_torques_port not connected"
				<< RTT::endlog();
		return false;
	}
	return true;
}

void PositionController::updateHook() {
	// cartesian task
	in_desiredTaskSpacePosition_flow = in_desiredTaskSpacePosition_port.read(
			in_desiredTaskSpacePosition_var);
	in_desiredTaskSpaceVelocity_flow = in_desiredTaskSpaceVelocity_port.read(
			in_desiredTaskSpaceVelocity_var);
	in_desiredTaskSpaceAcceleration_flow =
			in_desiredTaskSpaceAcceleration_port.read(
					in_desiredTaskSpaceAcceleration_var);

	in_currentTaskSpacePosition_flow = in_currentTaskSpacePosition_port.read(
			in_currentTaskSpacePosition_var);
	in_currentTaskSpaceVelocity_flow = in_currentTaskSpaceVelocity_port.read(
			in_currentTaskSpaceVelocity_var);
	in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);

	in_jacobian_flow = in_jacobian_port.read(in_jacobian_var);
	in_jacobianDot_flow = in_jacobianDot_port.read(in_jacobianDot_var);
	in_constraintLambda_flow = in_constraintLambda_port.read(
			in_constraintLambda_var);
	in_h_flow = in_h_port.read(in_h_var);
	in_constraintM_flow = in_constraintM_port.read(in_constraintM_var);
	in_P_flow = in_P_port.read(in_P_var);
	in_constraintC_flow = in_constraintC_port.read(in_constraintC_var);

	if (in_desiredTaskSpacePosition_flow == RTT::NoData
			|| in_desiredTaskSpaceVelocity_flow == RTT::NoData
			|| in_desiredTaskSpaceAcceleration_flow == RTT::NoData
			|| in_currentTaskSpacePosition_flow == RTT::NoData
			|| in_currentTaskSpaceVelocity_flow == RTT::NoData
			|| in_robotstatus_flow == RTT::NoData
			|| in_jacobian_flow == RTT::NoData
			|| in_jacobianDot_flow == RTT::NoData
			|| in_constraintLambda_flow == RTT::NoData
			|| in_h_flow == RTT::NoData || in_constraintM_flow == RTT::NoData
			|| in_P_flow == RTT::NoData || in_constraintC_flow == RTT::NoData) {
		return;
	}
	if (!receiveTranslationOnly) {
        if (true) {
        //position feedback part
//		rotx.angle() = in_desiredTaskSpacePosition_var(3);
//		roty.angle() = in_desiredTaskSpacePosition_var(4);
//		rotz.angle() = in_desiredTaskSpacePosition_var(5);
//		quat_target = rotx * roty * rotz;
        euler_temp = in_desiredTaskSpacePosition_var.tail<3>();
		toQuaternion(euler_temp, quat_target);
		quat_target.normalize();
//		rotx.angle() = in_currentTaskSpacePosition_var(3);
//		roty.angle() = in_currentTaskSpacePosition_var(4);
//		rotz.angle() = in_currentTaskSpacePosition_var(5);
//		quat_current = rotx * roty * rotz;
        euler_temp = in_currentTaskSpacePosition_var.tail<3>();
		toQuaternion(euler_temp, quat_current);
		quat_current.normalize();
		quat_diff = quat_target * (quat_current.inverse());
		quat_diff.normalize();

        toEulerAngles(euler_diff_pos, quat_diff);

        //velocity feedback part
//		rotx.angle() = in_desiredTaskSpaceVelocity_var(3);
//		roty.angle() = in_desiredTaskSpaceVelocity_var(4);
//		rotz.angle() = in_desiredTaskSpaceVelocity_var(5);
//		quat_target = rotx * roty * rotz;
        euler_temp = in_desiredTaskSpaceVelocity_var.tail<3>();
		toQuaternion(euler_temp, quat_target);
		quat_target.normalize();
//		rotx.angle() = in_currentTaskSpaceVelocity_var(3);
//		roty.angle() = in_currentTaskSpaceVelocity_var(4);
//		rotz.angle() = in_currentTaskSpaceVelocity_var(5);
//		quat_current = rotx * roty * rotz;
        euler_temp = in_currentTaskSpaceVelocity_var.tail<3>();
				toQuaternion(euler_temp, quat_current);
		quat_current.normalize();
		quat_diff = quat_target * (quat_current.inverse());
		quat_diff.normalize();

		toEulerAngles(euler_diff_vel, quat_diff);
        euler_diff_vel = in_desiredTaskSpaceVelocity_var.tail<3>() - in_currentTaskSpaceVelocity_var.tail<3>(); //TODO: correct?
        }
        else{
            //position feedback part
            euler_temp = in_desiredTaskSpacePosition_var.tail<3>();
            ExpEuler2Quaternion(euler_temp, quat_target);

            euler_temp = in_currentTaskSpacePosition_var.tail<3>();
            ExpEuler2Quaternion(euler_temp, quat_current);

            QuaternionProduct(quat_target, quat_current, quat_diff);

            LogQuaternion2Euler(quat_diff, euler_diff_pos);
            euler_diff_pos *= 2;

            //velocity feedback part
            euler_temp = in_desiredTaskSpaceVelocity_var.tail<3>();
            ExpEuler2Quaternion(euler_temp, quat_target);

            euler_temp = in_currentTaskSpaceVelocity_var.tail<3>();
            ExpEuler2Quaternion(euler_temp, quat_current);

            QuaternionProduct(quat_target, quat_current, quat_diff);

            LogQuaternion2Euler(quat_diff, euler_diff_vel);
            euler_diff_vel *= 2;
            euler_diff_vel = in_desiredTaskSpaceVelocity_var.tail<3>() - in_currentTaskSpaceVelocity_var.tail<3>(); //TODO: correct?
        }
	}

	error_pos =
			gainP
					* (in_desiredTaskSpacePosition_var
							- in_currentTaskSpacePosition_var);
	error_vel =
			gainD
					* (in_desiredTaskSpaceVelocity_var
							- in_currentTaskSpaceVelocity_var);
	if (!receiveTranslationOnly) {
        error_pos.tail<3>() = gainP_o * euler_diff_pos;
        error_vel.tail<3>() = gainD_o * euler_diff_vel;


	}
	//std::cout << (in_desiredTaskSpacePosition_var- in_currentTaskSpacePosition_var) << "\n----------------------------\n";


	// reference acceleration for cartesian task
	ref_Acceleration = in_desiredTaskSpaceAcceleration_var + (error_pos)
			+ (error_vel);

	//Start Khatib projected endeffector motion controller
	constraintForce = in_constraintLambda_var * ref_Acceleration;
	constraintForce += in_constraintLambda_var
			* (in_jacobian_var * in_constraintM_var.inverse() * in_P_var
					* in_h_var
					- (in_jacobianDot_var
							+ in_jacobian_var * in_constraintM_var.inverse()
									* in_constraintC_var)
							* in_robotstatus_var.velocities);

	out_torques_var.torques.setZero();
	out_torques_var.torques = in_jacobian_var.transpose() * constraintForce;
	out_torques_port.write(out_torques_var);
}

void PositionController::stopHook() {
}

void PositionController::cleanupHook() {
	portsArePrepared = false;
}

void PositionController::setDOFsize(unsigned int DOFsize) {
	assert(DOFsize > 0);
	this->DOFsize = DOFsize;
}

void PositionController::setTranslationOnly(const bool translationOnly) {
	receiveTranslationOnly = translationOnly;
	if (receiveTranslationOnly) {
		TaskSpaceDimension = 3;
	} else {
		TaskSpaceDimension = 6;
	}
}
void PositionController::setGains(float kp, float kd) {
	assert(kp >= 0);
	assert(kd >= 0);
	gainP = kp;
	gainD = kd;
}

void PositionController::setGainsOrientation(float kp, float kd) {
	assert(kp >= 0);
	assert(kd >= 0);
	gainP_o = kp;
	gainD_o = kd;
}

void PositionController::preparePorts() {
	if (portsArePrepared) {
		ports()->removePort("in_desiredTaskSpacePosition_port");  //1
		ports()->removePort("in_desiredTaskSpaceVelocity_port");  //2
		ports()->removePort("in_desiredTaskSpaceAcceleration_port");  //3

		ports()->removePort("in_currentTaskSpacePosition_port");  //4
		ports()->removePort("in_currentTaskSpaceVelocity_port");  //5
		ports()->removePort("in_robotstatus_port");  //6

		ports()->removePort("in_jacobian_port");    //12
		ports()->removePort("in_jacobianDot_port");     //13
		ports()->removePort("in_constraintLambda_port");    //14
		ports()->removePort("in_h_port");   //15
		ports()->removePort("in_constraintM_port");     //16
		ports()->removePort("in_P_port");   //17
		ports()->removePort("in_constraintC_port");     //18

		ports()->removePort("out_torques_port");

	}

	//prepare input
	//1
	in_desiredTaskSpacePosition_var = Eigen::VectorXf(TaskSpaceDimension);
	in_desiredTaskSpacePosition_port.setName(
			"in_desiredTaskSpacePosition_port");
	in_desiredTaskSpacePosition_port.doc(
			"Input port for reading the desired position from Trajectory Generator");
	ports()->addPort(in_desiredTaskSpacePosition_port);
	in_desiredTaskSpacePosition_flow = RTT::NoData;
	//2
	in_desiredTaskSpaceVelocity_var = Eigen::VectorXf(TaskSpaceDimension);
	in_desiredTaskSpaceVelocity_port.setName(
			"in_desiredTaskSpaceVelocity_port");
	in_desiredTaskSpaceVelocity_port.doc(
			"Input port for reading the desired velocity from Trajectory Generator");
	ports()->addPort(in_desiredTaskSpaceVelocity_port);
	in_desiredTaskSpaceVelocity_flow = RTT::NoData;
	//3
	in_desiredTaskSpaceAcceleration_var = Eigen::VectorXf(TaskSpaceDimension);
	in_desiredTaskSpaceAcceleration_port.setName(
			"in_desiredTaskSpaceAcceleration_port");
	in_desiredTaskSpaceAcceleration_port.doc(
			"Input port for reading the desired acceleration from Trajectory Generator");
	ports()->addPort(in_desiredTaskSpaceAcceleration_port);
	in_desiredTaskSpaceAcceleration_flow = RTT::NoData;
	//4
	in_currentTaskSpacePosition_var = Eigen::VectorXf(TaskSpaceDimension);
	in_currentTaskSpacePosition_port.setName(
			"in_currentTaskSpacePosition_port");
	in_currentTaskSpacePosition_port.doc(
			"Input port for reading the current position");
	ports()->addPort(in_currentTaskSpacePosition_port);
	in_currentTaskSpacePosition_flow = RTT::NoData;
	//5
	in_currentTaskSpaceVelocity_var = Eigen::VectorXf(TaskSpaceDimension);
	in_currentTaskSpaceVelocity_port.setName(
			"in_currentTaskSpaceVelocity_port");
	in_currentTaskSpaceVelocity_port.doc(
			"Input port for reading the current position velocity");
	ports()->addPort(in_currentTaskSpaceVelocity_port);
	in_currentTaskSpaceVelocity_flow = RTT::NoData;
	//6
	in_robotstatus_var = rstrt::robot::JointState(DOFsize);
	in_robotstatus_port.setName("in_robotstatus_port");
	in_robotstatus_port.doc("Input port for reading the joint velocity");
	ports()->addPort(in_robotstatus_port);
	in_robotstatus_flow = RTT::NoData;
	//12
	in_jacobian_var = Eigen::MatrixXf(TaskSpaceDimension, DOFsize);
	in_jacobian_port.setName("in_jacobian_port");
	in_jacobian_port.doc("Input port for reading the jacobian");
	ports()->addPort(in_jacobian_port);
	in_jacobian_flow = RTT::NoData;
	//13
	in_jacobianDot_var = Eigen::MatrixXf(TaskSpaceDimension, DOFsize);
	in_jacobianDot_port.setName("in_jacobianDot_port");
	in_jacobianDot_port.doc(
			"Input port for reading the derivative of jacobian");
	ports()->addPort(in_jacobianDot_port);
	in_jacobianDot_flow = RTT::NoData;
	//14
	in_constraintLambda_var = Eigen::MatrixXf(TaskSpaceDimension,
			TaskSpaceDimension);
	in_constraintLambda_port.setName("in_constraintLambda_port");
	in_constraintLambda_port.doc(
			"Input port for reading the constraint Lambda (after Eq. (11) of the paper)");
	ports()->addPort(in_constraintLambda_port);
	in_constraintLambda_flow = RTT::NoData;
	//15
	in_h_var = Eigen::VectorXf(DOFsize);
	in_h_port.setName("in_h_port");
	in_h_port.doc(
			"Input port for reading the constraint Lambda (under Eq. (11) of the paper)");
	ports()->addPort(in_h_port);
	in_h_flow = RTT::NoData;
	//16
	in_constraintM_var = Eigen::MatrixXf(DOFsize, DOFsize);
	in_constraintM_port.setName("in_constraintM_port");
	in_constraintM_port.doc(
			"Input port for reading the constraint inertia matrix (under Eq. (8) of the paper)");
	ports()->addPort(in_constraintM_port);
	in_constraintM_flow = RTT::NoData;
	//17
	in_P_var = Eigen::MatrixXf(DOFsize, DOFsize);
	in_P_port.setName("in_P_port");
	in_P_port.doc("Input port for reading the projection matrix");
	ports()->addPort(in_P_port);
	in_P_flow = RTT::NoData;
	//18
	in_constraintC_var = Eigen::MatrixXf(DOFsize, DOFsize);
	in_constraintC_port.setName("in_constraintC_port");
	in_constraintC_port.doc("Input port for reading the constraint C matrix");
	ports()->addPort(in_constraintC_port);
	in_constraintC_flow = RTT::NoData;

	//prepare output
	out_torques_var = rstrt::dynamics::JointTorques(DOFsize);
	out_torques_var.torques.setZero();
	out_torques_port.setName("out_torques_port");
	out_torques_port.doc("Output port for sending torque values");
	out_torques_port.setDataSample(out_torques_var);
	ports()->addPort(out_torques_port);

	error_pos.resize(TaskSpaceDimension);
	error_vel.resize(TaskSpaceDimension);
	portsArePrepared = true;
}

void PositionController::displayStatus() {
	RTT::log(RTT::Info) << "in_desiredTaskSpacePosition_var \n"
			<< in_desiredTaskSpacePosition_var << RTT::endlog();
	RTT::log(RTT::Info) << "in_desiredTaskSpaceVelocity_var \n"
			<< in_desiredTaskSpaceVelocity_var << RTT::endlog();
	RTT::log(RTT::Info) << "in_desiredTaskSpaceAcceleration_var \n"
			<< in_desiredTaskSpaceAcceleration_var << RTT::endlog();

	RTT::log(RTT::Info) << "in_currentTaskSpacePosition_var \n"
			<< in_currentTaskSpacePosition_var << RTT::endlog();
	RTT::log(RTT::Info) << "in_currentTaskSpaceVelocity_var \n"
			<< in_currentTaskSpaceVelocity_var << RTT::endlog();

	RTT::log(RTT::Info) << "in_jacobian_var \n" << in_jacobian_var
			<< RTT::endlog();
	RTT::log(RTT::Info) << "in_jacobianDot_var \n" << in_jacobianDot_var
			<< RTT::endlog();
	RTT::log(RTT::Info) << "in_constraintLambda_var \n"
			<< in_constraintLambda_var << RTT::endlog();
	RTT::log(RTT::Info) << "in_h_var \n" << in_h_var << RTT::endlog();
	RTT::log(RTT::Info) << "in_constraintM_var \n" << in_constraintM_var
			<< RTT::endlog();
	RTT::log(RTT::Info) << "in_P_var \n" << in_P_var << RTT::endlog();
	RTT::log(RTT::Info) << "in_constraintC_var \n" << in_constraintC_var
			<< RTT::endlog();
	RTT::log(RTT::Info) << "out_torques_var \n" << out_torques_var.torques
			<< RTT::endlog();
}

void PositionController::toEulerAngles(Eigen::Vector3f& res,
        Eigen::Quaternionf const & quat) const {
	res(0) = atan2(2 * ((quat.w() * quat.x()) + (quat.y() * quat.z())),
			1 - (2 * ((quat.x() * quat.x()) + (quat.y() * quat.y()))));
	res(1) = asin(2 * ((quat.w() * quat.y()) - (quat.z() * quat.x())));
	res(2) = atan2(2 * ((quat.w() * quat.z()) + (quat.x() * quat.y())),
			1 - (2 * ((quat.y() * quat.y()) + (quat.z() * quat.z()))));
}
void PositionController::toQuaternion(Eigen::Vector3f const & rpy,
		Eigen::Quaternionf& res) const {
	res.x() = (sin(rpy.x() / 2) * cos(rpy.y() / 2) * cos(rpy.z() / 2))
			- (cos(rpy.x() / 2) * sin(rpy.y() / 2) * sin(rpy.z() / 2));
	res.y() = (cos(rpy.x() / 2) * sin(rpy.y() / 2) * cos(rpy.z() / 2))
			+ (sin(rpy.x() / 2) * cos(rpy.y() / 2) * sin(rpy.z() / 2));
	res.z() = (cos(rpy.x() / 2) * cos(rpy.y() / 2) * sin(rpy.z() / 2))
			- (sin(rpy.x() / 2) * sin(rpy.y() / 2) * cos(rpy.z() / 2));
	res.w() = (cos(rpy.x() / 2) * cos(rpy.y() / 2) * cos(rpy.z() / 2))
			+ (sin(rpy.x() / 2) * sin(rpy.y() / 2) * sin(rpy.z() / 2));
}

void PositionController::QuaternionProduct(
        Eigen::Quaternionf const & quat1,
        Eigen::Quaternionf const & quat2,
        Eigen::Quaternionf& quatResult) {

    float quaternionV1 = quat1.w();
    Eigen::Vector3f quaternionU1 = Eigen::Vector3f();
    quaternionU1(0) = quat1.x();
    quaternionU1(1) = quat1.y();
    quaternionU1(2) = quat1.z();

    float quaternionV2 = quat2.w();
    Eigen::Vector3f quaternionU2 = Eigen::Vector3f();
    quaternionU2(0) = quat2.x();
    quaternionU2(1) = quat2.y();
    quaternionU2(2) = quat2.z();

    float quaternionVResult;
    Eigen::Vector3f quaternionUResult = Eigen::Vector3f();

    this->QuaternionProduct(quaternionV1, quaternionU1, quaternionV2, quaternionU2, quaternionVResult, quaternionUResult);

    quatResult.w() = quaternionVResult;
    quatResult.x() = quaternionUResult(0);
    quatResult.y() = quaternionUResult(1);
    quatResult.z() = quaternionUResult(2);
}

void PositionController::QuaternionProduct(
        float const & quaternionV1,
        Eigen::Vector3f const & quaternionU1,
        float const & quaternionV2,
        Eigen::Vector3f const & quaternionU2,
        float & resultV,
        Eigen::Vector3f & resultU) {

    resultV = 	quaternionV1 * quaternionV2 - quaternionU1.transpose() * quaternionU2;
    resultU = 	quaternionV1 * quaternionU2 +
                quaternionV2 * quaternionU1 +
                quaternionU1.cross(quaternionU2);
}

void PositionController::ExpEuler2Quaternion(Eigen::Vector3f const & euler, Eigen::Quaternionf & quaternion) {
    float quaternionV;
    Eigen::Vector3f quaternionU = Eigen::Vector3f();

    this->ExpEuler2Quaternion(euler, quaternionV, quaternionU);

    quaternion.w() = quaternionV;
    quaternion.x() = quaternionU(0);
    quaternion.y() = quaternionU(1);
    quaternion.z() = quaternionU(2);
}

void PositionController::ExpEuler2Quaternion(Eigen::Vector3f const & euler, float & quaternionV, Eigen::Vector3f & quaternionU) {
    // equals quaternion = exp(euler)

    if (euler.norm() > 3.14){
        RTT::log(RTT::Info) << "norm(euler) > pi \n" << euler.norm() << RTT::endlog();
    }

    if(euler(0) == 0.0 && euler(1) == 0.0 && euler(2) == 0.0){
        quaternionV = 1.0;
        quaternionU.setZero();
    }
    else{
        quaternionV = cos(euler.norm());
        quaternionU = sin(euler.norm()) * euler.transpose() / euler.norm();

        //or use .squaredNorm() here??
//		quaternionV = cos(euler.squaredNorm());
//		quaternionU = sin(euler.squaredNorm()) * euler.transpose() / euler.squaredNorm();
    }
}

void PositionController::LogQuaternion2Euler(Eigen::Quaternionf const & quaternion, Eigen::Vector3f & euler) {
    float quaternionV = quaternion.w();
    Eigen::Vector3f quaternionU = Eigen::Vector3f();
    quaternionU(0) = quaternion.x();
    quaternionU(1) = quaternion.y();
    quaternionU(2) = quaternion.z();

    this->LogQuaternion2Euler(quaternionV, quaternionU, euler);
}

void PositionController::LogQuaternion2Euler(float const & quaternionV, Eigen::Vector3f const & quaternionU, Eigen::Vector3f & euler) {
    // equals euler = log(quaternion)

    if(quaternionU(0) == 0.0 && quaternionU(1) == 0.0 && quaternionU(2) == 0.0){
    //if(quaternionU.norm() == 0.0){
        euler.setZero();
    }
    else{
        euler = acos(quaternionV) * quaternionU / quaternionU.norm();

        //or use .squaredNorm() here??
//		euler = acos(quaternionV) * quaternionU / quaternionU.squaredNorm();
    }
}


//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(PositionController)

