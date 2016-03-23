#ifndef RTT_LWR_OSF_CONTROLLER_HPP
#define RTT_LWR_OSF_CONTROLLER_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/base/RunnableInterface.hpp>
#include <rtt/Activity.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

#include <rci/dto/JointAngles.h>
#include <rci/dto/JointTorques.h>
#include <rci/dto/JointVelocities.h>
#include <rci/dto/JointAccelerations.h>
#include <rci/dto/JointImpedance.h>
#include <rci/dto/CartesianPose.h>

#include <rsb/Factory.h>
#include <rsb/Listener.h>

#include "../baseclasses/RTTArmControllerBase.hpp"
#include "../parsertools/KDLParser.hpp"

#include "CartesianSpace_CircularTask.hpp"
#include "TaskTest.hpp"
#include "JointTaskTest.hpp"
#include "QuinticPolynomial.hpp"
#include "FileWriterCSV.hpp"
#include "OrientationHelper.hpp"

#define DEFAULT_ROOT_LINK "lwr_arm_base_link"
#define DEFAULT_TIP_LINK "lwr_arm_7_link"
#define DEFAULT_NR_JOINTS_LWR 7

boost::mutex rsbcmdJointAngles_mutex;
rci::JointAnglesPtr rsbcmdJointAngles_store;
RTT::FlowStatus rsbCMDJntPos_Flow;

class RttLwrOSFController: public RTTArmControllerBase {
public:
	RttLwrOSFController(std::string const& name);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

protected:
	/**
	 * OutputPorts publish data.
	 */
	RTT::OutputPort<rci::JointAnglesPtr> cmdJntPos_Port;
	RTT::OutputPort<rci::JointTorquesPtr> cmdJntTrq_Port;
	// hack TODO
	RTT::OutputPort<rci::JointImpedancePtr> cmdJntImp_Port;
	/**
	 * InputPorts read data.
	 */
	RTT::InputPort<rci::JointAnglesPtr> currJntPos_Port;
	RTT::FlowStatus currJntPos_Flow;
	RTT::InputPort<rci::JointVelocitiesPtr> currJntVel_Port;
	RTT::FlowStatus currJntVel_Flow;
	RTT::InputPort<rci::JointTorquesPtr> currJntTrq_Port;
	RTT::FlowStatus currJntTrq_Flow;
	/**
	 * Hold the read value
	 */
	rci::JointAnglesPtr currJntPos;
	rci::JointTorquesPtr currJntTrq;
	rci::JointVelocitiesPtr currJntVel;
	Eigen::VectorXd lastJntVel;
	Eigen::VectorXd currJntAcc;

	//start variables for quaternion feedback stuff
	rci::OrientationPtr desiredCartOrientation;
	rci::OrientationPtr currCartOrientation;
	double desiredCartOrientationQuaternionV; //could be also double
	Eigen::Vector3d desiredCartOrientationQuaternionU;
	double currCartOrientationQuaternionV; //could be also double
	Eigen::Vector3d currCartOrientationQuaternionU;
	double QuaternionProductV; //could be also double
	Eigen::Vector3d QuaternionProductU;
	Eigen::Vector3d QuaternionProductEuler;
	//stop variables for quaternion feedback stuff

	/**
	 * Hold the write value
	 */
	rci::JointAnglesPtr outJntPos;
	rci::JointTorquesPtr outJntTrq;

	// converted commands from rsb
	Eigen::VectorXd rsbCMDJntPos;
	Eigen::VectorXd rsbCMDJntImp;

	/**
	 * Commands from the outside world
	 */
	rsb::ListenerPtr rsbcmdJointPosition_Listener;
	/**
	 * Misc controller vars
	 */
	double gain;
	double last_SimulationTime;

	/** ####### STORAGE FIELDS FOR CONTROLLERS ####### */
	Eigen::VectorXd jnt_trq_cmd_;
	Eigen::VectorXd jnt_trq_cmd_Motion_Khatib, jnt_trq_cmd_Nullspace_Khatib, jnt_trq_cmd_Force_Khatib;
	Eigen::VectorXd jnt_trq_cmd_Motion_Projected, jnt_trq_cmd_Nullspace_Projected, jnt_trq_cmd_Force_Projected;

	// Control gain
    Eigen::VectorXd kg_;

	Eigen::VectorXd h;

	TaskTest task_;
    JointTaskTest joint_task_;
	Eigen::VectorXd yD;

    double getSimulationTime();

    OrientationHelper orientation_helper;
    FileWriterCSV * csv_logger;
    QuinticPolynomial QP;
    TaskTest          _task_test;
    CartesianSpace_CircularTask cart_task;
    Eigen::VectorXd q_p;
    double start_time;
    KDL::JntArray q_des_Nullspace;
    KDL::JntArray q_des_FirstPoint;
    KDL::JntArray task_q;
    KDL::JntArray task_qd;
    KDL::JntArray task_qdd;
    KDL::JntArray task_p;
    KDL::JntArray task_pd;
    KDL::JntArray task_pdd;
    Eigen::Vector3d task_pTranslation;
    Eigen::Vector3d task_pdTranslation;
    Eigen::Vector3d task_pddTranslation;
    Eigen::Vector3d task_pOrientation;
    Eigen::Vector3d task_pdOrientation;
    Eigen::Vector3d task_pddOrientation;
    KDL::JntArray rne_torques;
    KDL::Wrenches ext_force;
    Eigen::VectorXd Kp_joint, Kd_joint;
    Eigen::VectorXd Kp_cartTranslation, Kd_cartTranslation;
    Eigen::VectorXd Kp_cartOrientationEuler, Kd_cartOrientationEuler;
    Eigen::VectorXd Kp_cartOrientationQuaternion, Kd_cartOrientationQuaternion;
    Eigen::VectorXd curr_ee_pose, curr_ee_vel;
	Eigen::Vector3d curr_ee_poseTranslation, curr_ee_poseOrientation;
	Eigen::Vector3d curr_ee_velTranslation, curr_ee_velOrientation;

    double safety_margin_Pos, safety_margin_Vel;
    Eigen::VectorXd jointPosLimits_max, jointPosLimits_min, jointPosLimits_range, jointPosCritic_max, jointPosCritic_min;
    Eigen::VectorXd jointVelLimits_max, jointVelLimits_min, jointVelLimits_range, jointVelCritic_max, jointVelCritic_min;

    //Khatib controller:
	Eigen::MatrixXd Lamda, Lamda_cstr;
	Eigen::VectorXd CG_bar;
	Eigen::VectorXd Forces, Forces_cstr;

	Eigen::MatrixXd jac_cstr_;
	Eigen::MatrixXd jac_cstr_MPI;
	KDL::JntArrayVel joint_position_velocity_des;

	Eigen::VectorXd tau_0;
	Eigen::VectorXd init, final;
	Eigen::VectorXd Pi, Pf;

	KDL::JntArray q_from_robot;
	KDL::JntArray qd_from_robot;
	Eigen::VectorXd ref_acc, ref_accTranslation, ref_accOrientation;
	Eigen::MatrixXd tmpeye77;
	Eigen::MatrixXd tmpeye66;
	Eigen::MatrixXd preLambda;
	Eigen::VectorXd lambda_des;
	Eigen::MatrixXd P;
	Eigen::MatrixXd N;

	Eigen::MatrixXd identity77, identity66, identity33;
	Eigen::MatrixXd diagonal66, diagonal33;


	Eigen::MatrixXd M_cstr_;
	Eigen::MatrixXd C_cstr_;

    Eigen::VectorXd getQFromGazebo_EIGEN();
    Eigen::VectorXd getQdFromGazebo_EIGEN();
    Eigen::VectorXd getQddFromGazebo_EIGEN();
    KDL::JntArray   getQFromGazebo_KDL();
    KDL::JntArray   getQdFromGazebo_KDL();
    KDL::JntArray   getQddFromGazebo_KDL();

    double internalStartTime;
    bool detectedError;

    // orocos attributes
    std::vector<double> tenGains;

    double Kp_cartTranslationKhatibGain;
    double Kd_cartTranslationKhatibGain;
    double Kp_cartOrientationEulerKhatibGain;
    double Kd_cartOrientationEulerKhatibGain;
    double Kp_cartOrientationQuaternionKhatibGain;
	double Kd_cartOrientationQuaternionKhatibGain;
    double Kp_jointKhatibGain;
    double Kd_jointKhatibGain;

    double Kp_cartTranslationConstrainedGain;
    double Kd_cartTranslationConstrainedGain;
    double Kp_cartOrientationEulerConstrainedGain;
    double Kd_cartOrientationEulerConstrainedGain;
    double Kp_cartOrientationQuaternionConstrainedGain;
	double Kd_cartOrientationQuaternionConstrainedGain;
    double Kp_jointConstrainedGain;
    double Kd_jointConstrainedGain;

    bool use_original_khatib_controller;
    bool use_euler_orientation;

//    Eigen::MatrixXd inverseDynamicsTorques(KDL::JntSpaceInertiaMatrix & _inertia, KDL::JntArray & _coriolis, KDL::JntArray & _gravity, Eigen::VectorXd Kp, Kd);
    //EOP

};
#endif
