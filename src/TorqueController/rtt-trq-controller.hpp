#ifndef RTT_TRQ_CONTROLLER_HPP
#define RTT_TRQ_CONTROLLER_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/base/RunnableInterface.hpp>
#include <rtt/Activity.hpp>

#include <Eigen/Dense>

#include <rci/dto/JointAngles.h>
#include <rci/dto/JointTorques.h>
#include <rci/dto/JointVelocities.h>
#include <rci/dto/JointAccelerations.h>
#include <rci/dto/JointImpedance.h>

#include <rsb/Factory.h>
#include <rsb/Listener.h>

#include "../baseclasses/RTTArmControllerBase.hpp"
#include "../parsertools/KDLParser.hpp"

#include "TaskTest.hpp"
#include "JointTaskTest.hpp"
#include "QuinticPolynomial.hpp"


#define DEFAULT_ROOT_LINK "lwr_arm_base_link"
#define DEFAULT_TIP_LINK "lwr_arm_7_link"
#define DEFAULT_NR_JOINTS_LWR 7

boost::mutex rsbcmdJointAngles_mutex;
rci::JointAnglesPtr rsbcmdJointAngles_store;
RTT::FlowStatus rsbCMDJntPos_Flow;

class RTTTrqController: public RTTArmControllerBase {
public:
	RTTTrqController(std::string const& name);
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
	rci::JointVelocitiesPtr lastJntVel;
	rci::JointAccelerationsPtr currJntAcc;
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

    // Pouya
    double getSimulationTime();
    QuinticPolynomial QP;
    TaskTest          _task_test;
    Eigen::VectorXd q_p;
    double start_time;
    KDL::JntArray q_des_Nullspace;
    KDL::JntArray q_des_FirstPoint;
    KDL::JntArray q_tmp;
    KDL::JntArray qd_tmp;
    KDL::JntArray qdd_tmp;
    KDL::JntArray p_tmp;
    KDL::JntArray pd_tmp;
    KDL::JntArray pdd_tmp;
    KDL::JntArray rne_torques;
    KDL::Wrenches ext_force;
    Eigen::VectorXd Kp_joint, Kd_joint;
    Eigen::VectorXd Kp_cart, Kd_cart;
    Eigen::VectorXd curr_ee_pose, curr_ee_vel;

    //Khatib controller:
	Eigen::MatrixXd Lamda, Lamda_cstr;
	Eigen::VectorXd CG_bar;
	Eigen::VectorXd Forces, Forces_cstr;

//	KDL::Jacobian _jac, _jac_dot;
	Eigen::MatrixXd jac_cstr_;
	Eigen::MatrixXd jac_cstr_MPI;
	KDL::JntArrayVel joint_position_velocity_des;

	Eigen::VectorXd tau_0;

	KDL::JntArray q_from_robot;
	KDL::JntArray qd_from_robot;
	Eigen::VectorXd ref_acc;
	Eigen::MatrixXd tmpeye77;
	Eigen::MatrixXd tmpeye66;
	Eigen::MatrixXd preLambda;
	Eigen::VectorXd lambda_des;
	Eigen::MatrixXd P;
	Eigen::MatrixXd N;

	Eigen::MatrixXd identity77;
	Eigen::MatrixXd identity66;


	Eigen::MatrixXd M_cstr_;
	Eigen::MatrixXd C_cstr_;

    Eigen::VectorXd getQFromGazebo_EIGEN();
    Eigen::VectorXd getQdFromGazebo_EIGEN();
    Eigen::VectorXd getQddFromGazebo_EIGEN();
    KDL::JntArray   getQFromGazebo_KDL();
    KDL::JntArray   getQdFromGazebo_KDL();
    KDL::JntArray   getQddFromGazebo_KDL();

    double internalStartTime;

//    Eigen::MatrixXd inverseDynamicsTorques(KDL::JntSpaceInertiaMatrix & _inertia, KDL::JntArray & _coriolis, KDL::JntArray & _gravity, Eigen::VectorXd Kp, Kd);
    //EOP

};
#endif
