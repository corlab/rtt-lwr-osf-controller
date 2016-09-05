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
    addOperation("setConstrainedVersionMode", &PositionController::setConstrainedVersionMode, this).doc(
            "set constrained version mode");
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
    this->setConstrainedVersionMode(true);
    this->setTranslationOnly(true);

	rotx = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX());
	roty = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());
	rotz = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
//	rotx.axis() = Eigen::Vector3f::UnitX();
//	roty.axis() = Eigen::Vector3f::UnitY();
//	rotz.axis() = Eigen::Vector3f::UnitZ();

    quaternion_desired = Eigen::Vector4f::Zero();
    quaternion_current = Eigen::Vector4f::Zero();
    quaternion_current_conj = Eigen::Vector4f::Zero();
    quaternion_diff = Eigen::Vector4f::Zero();

    quat_target = Eigen::Quaternionf();
    quat_current = Eigen::Quaternionf();
    quat_diff = Eigen::Quaternionf();

    error_o_pos = Eigen::Vector3f::Zero();
    error_o_vel = Eigen::Vector3f::Zero();
    euler_temp = Eigen::Vector3f::Zero();
    axisangle_temp = Eigen::Vector3f::Zero();
    qh = QuaternionHelper();
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
    if (!in_desiredTaskSpaceVelocity_port.connected()) {
        RTT::log(RTT::Info) << "in_desiredTaskSpaceVelocity_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_desiredTaskSpaceAcceleration_port.connected()) {
        RTT::log(RTT::Info) << "in_desiredTaskSpaceAcceleration_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_currentTaskSpacePosition_port.connected()) {
        RTT::log(RTT::Info) << "in_currentTaskSpacePosition_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_currentTaskSpaceVelocity_port.connected()) {
        RTT::log(RTT::Info) << "in_currentTaskSpaceVelocity_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_jacobian_port.connected()) {
        RTT::log(RTT::Info) << "in_jacobian_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_jacobianDot_port.connected()) {
        RTT::log(RTT::Info) << "in_jacobianDot_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_constraintLambda_port.connected()) {
        RTT::log(RTT::Info) << "in_constraintLambda_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_h_port.connected()) {
        RTT::log(RTT::Info) << "in_h_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_constraintM_port.connected()) {
        RTT::log(RTT::Info) << "in_constraintM_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_P_port.connected()) {
        RTT::log(RTT::Info) << "in_P_port not connected"
                << RTT::endlog();
        return false;
    }
    if (!in_constraintC_port.connected()) {
        RTT::log(RTT::Info) << "in_constraintC_port not connected"
                << RTT::endlog();
        return false;
    }
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
        if (false) {
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
//			std::cout<<quat_current.w()<<", "<<quat_current.x()<<", "<<quat_current.y()<<", "<<quat_current.z()<<", norm:"<<quat_current.norm()<<" END"<<"\n------------------\n";
			quat_diff = quat_target * (quat_current.inverse());
			quat_diff.normalize();

            toEulerAngles(error_o_pos, quat_diff);

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

            toEulerAngles(error_o_vel, quat_diff);
		} else {
            error_o_pos.setZero();
            error_o_vel.setZero();

            //position feedback part
            //version 1
//            axisangle_temp = in_desiredTaskSpacePosition_var.tail<3>();
//            qh.AxisAngle2Quaternion(axisangle_temp, quaternion_desired);

//            axisangle_temp = in_currentTaskSpacePosition_var.tail<3>();
//            qh.AxisAngle2Quaternion(axisangle_temp, quaternion_current);

//            Eigen::MatrixXf skewmat;
//            skewmat = Eigen::MatrixXf::Zero(3,3);
//            skewmat(0,1) = - quaternion_desired(3);
//            skewmat(0,2) = + quaternion_desired(2);
//            skewmat(1,2) = - quaternion_desired(1);

//            skewmat(1,0) = + quaternion_desired(3);
//            skewmat(2,0) = - quaternion_desired(2);
//            skewmat(2,1) = + quaternion_desired(1);

//            error_o_pos = quaternion_desired(0) * quaternion_current.tail<3>()
//                             - quaternion_current(0) * quaternion_desired.tail<3>()
//                             + skewmat * quaternion_current.tail<3>();

            //version 2
            axisangle_temp = in_desiredTaskSpacePosition_var.tail<3>();
            qh.ExpAxisAngle2Quaternion(axisangle_temp, quaternion_desired);

            axisangle_temp = in_currentTaskSpacePosition_var.tail<3>();
            qh.ExpAxisAngle2Quaternion(axisangle_temp, quaternion_current);

            qh.ConjugateQuaternion(quaternion_current, quaternion_current_conj);

            qh.QuaternionProduct(quaternion_desired, quaternion_current_conj, quaternion_diff);

            float n=pow(quaternion_diff(0),2) + pow(quaternion_diff(1),2) + pow(quaternion_diff(2),2) + pow(quaternion_diff(3),2);
            if (n > 1.1){
                std::cout << " norm(quaternion_diff) = " << n << std::endl;
            }
            if (n < 0.9){
                std::cout << " norm(quaternion_diff) = " << n << std::endl;
            }

            qh.LogQuaternion2AxisAngle(quaternion_diff, error_o_pos);
            error_o_pos *= 2;

            //velocity feedback part
//            euler_temp = in_desiredTaskSpaceVelocity_var.tail<3>();
//            ExpAxisAngle2Quaternion(euler_temp, quaternion_desired);

//            euler_temp = in_currentTaskSpaceVelocity_var.tail<3>();
//            ExpAxisAngle2Quaternion(euler_temp, quaternion_current);

//            ConjugateQuaternion(quaternion_current, quaternion_current_conj);

//            QuaternionProduct(quaternion_desired, quaternion_current_conj, quaternion_diff);

//            LogQuaternion2AxisAngle(quaternion_diff, error_o_vel);
//            error_o_vel *= 2;

//            error_o_vel = in_desiredTaskSpaceVelocity_var.tail<3>()
//                          - in_currentTaskSpaceVelocity_var.tail<3>(); //TODO: correct?

//            error_o_vel = - euler_temp;
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
        error_pos.tail<3>() = gainP_o * error_o_pos;
        error_vel.tail<3>() = gainD_o * error_o_vel;

	}

	// reference acceleration for cartesian task
	ref_Acceleration = in_desiredTaskSpaceAcceleration_var + (error_pos)
			+ (error_vel);

	//Start Khatib projected endeffector motion controller
    if(useConstrainedVersion){
        constraintForce = in_constraintLambda_var * ref_Acceleration;
        constraintForce += in_constraintLambda_var
                * (in_jacobian_var * in_constraintM_var.inverse() * in_P_var
                        * in_h_var
                        - (in_jacobianDot_var
                                + in_jacobian_var * in_constraintM_var.inverse()
                                        * in_constraintC_var)
                                * in_robotstatus_var.velocities);
    }
    else{
        constraintForce = ((in_jacobian_var*in_constraintM_var.inverse()*in_jacobian_var.transpose()).inverse())*ref_Acceleration+
                          ((in_jacobian_var*in_constraintM_var.inverse()*in_jacobian_var.transpose()).inverse())*(-in_jacobianDot_var*in_robotstatus_var.velocities)+
                          (((in_jacobian_var*in_constraintM_var.inverse()*in_jacobian_var.transpose()).inverse())*in_jacobian_var*in_constraintM_var.inverse()*in_h_var);
    //    constraintForce = ref_Acceleration;
    }

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

void PositionController::setConstrainedVersionMode(bool useConstrainedVersion){
    this->useConstrainedVersion = useConstrainedVersion;
}

void PositionController::setTranslationOnly(const bool translationOnly) {
	receiveTranslationOnly = translationOnly;
	if (receiveTranslationOnly) {
		TaskSpaceDimension = 3;
	} else {
		TaskSpaceDimension = 6;
	}

    error_pos = Eigen::VectorXf(TaskSpaceDimension);
    error_vel = Eigen::VectorXf(TaskSpaceDimension);
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

//    RTT::log(RTT::Info) << "quaternion_current \n" << quaternion_current
//            << RTT::endlog();
//    RTT::log(RTT::Info) << "quaternion_desired \n" << quaternion_desired
//            << RTT::endlog();
//    RTT::log(RTT::Info) << "error_t_pos \n" << (in_desiredTaskSpacePosition_var.head<3>() - in_currentTaskSpacePosition_var.head<3>())
//            << RTT::endlog();
//    RTT::log(RTT::Info) << "error_t_vel \n" << (in_desiredTaskSpaceVelocity_var.head<3>() - in_currentTaskSpaceVelocity_var.head<3>())
//            << RTT::endlog();

    if(TaskSpaceDimension==6){
//        RTT::log(RTT::Info) << "error_o_pos direct \n" << (in_desiredTaskSpacePosition_var.tail<3>() - in_currentTaskSpacePosition_var.tail<3>())
//                << RTT::endlog();
//        RTT::log(RTT::Info) << "error_o_vel direct \n" << (in_desiredTaskSpaceVelocity_var.tail<3>() - in_currentTaskSpaceVelocity_var.tail<3>())
//                << RTT::endlog();

//        RTT::log(RTT::Info) << "error_o_pos \n" << error_o_pos
//                << RTT::endlog();
//        RTT::log(RTT::Info) << "error_o_vel \n" << error_o_vel
//                << RTT::endlog();

//        Eigen::VectorXf foobar = Eigen::VectorXf::Zero(6);
//        foobar(0) = 1.1;
//        foobar(1) = 2.2;
//        foobar(2) = 3.3;
//        foobar(3) = 4.4;
//        foobar(4) = 5.5;
//        foobar(5) = 6.6;

//        RTT::log(RTT::Info) << "foobar \n" << foobar << RTT::endlog();
//        RTT::log(RTT::Info) << "foobar.tail<3>() \n" << foobar.tail<3>() << RTT::endlog();
//        RTT::log(RTT::Info) << "foobar.tail(3) \n" << foobar.tail(3) << RTT::endlog();

        //axisangle_temp = in_desiredTaskSpacePosition_var.tail<3>();
//        axisangle_temp = in_currentTaskSpacePosition_var.tail<3>();
//        qh.test(axisangle_temp);
    }
}

void PositionController::toEulerAngles(Eigen::Vector3f& res,
		Eigen::Quaternionf const & quat) const {
//	res(0) = atan2(2 * ((quat.w() * quat.x()) + (quat.y() * quat.z())),
//			1 - (2 * ((quat.x() * quat.x()) + (quat.y() * quat.y()))));
//	res(1) = asin(2 * ((quat.w() * quat.y()) - (quat.z() * quat.x())));
//	res(2) = atan2(2 * ((quat.w() * quat.z()) + (quat.x() * quat.y())),
//			1 - (2 * ((quat.y() * quat.y()) + (quat.z() * quat.z()))));
	res(2) = atan2(-2*(quat.y()*quat.z() - quat.w()*quat.x()),quat.w()*quat.w() - quat.x()*quat.x() - quat.y()*quat.y() + quat.z()*quat.z());
	res(1) = asin ( 2*(quat.x()*quat.z() + quat.w()*quat.y()) );

	res(0) = atan2( -2*(quat.x()*quat.y() - quat.w()*quat.z()), quat.w()*quat.w() + quat.x()*quat.x() - quat.y()*quat.y() - quat.z()*quat.z());
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


//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(PositionController)

