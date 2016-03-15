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

	/** ####### STORAGE FIELDS FOR CONTROLLERS ####### */
	Eigen::VectorXd jnt_trq_cmd_;
	// Control gain
    Eigen::VectorXd kg_;

	Eigen::MatrixXd Lamda;
	Eigen::VectorXd h;
	Eigen::VectorXd F;

	TaskTest task_;
    JointTaskTest joint_task_;
	Eigen::VectorXd yD;

    // Pouya
    double getSimulationTime();
    QuinticPolynomial QP;
    TaskTest          _task_test;
    Eigen::VectorXd q_p;
    double start_time;
    KDL::JntArray _gravity;
    KDL::JntArray _coriolis;
    KDL::JntSpaceInertiaMatrix _inertia;
    KDL::JntArray q_tmp;
    KDL::JntArray qd_tmp;
    KDL::JntArray qdd_tmp;
    KDL::JntArray p_tmp;
    KDL::JntArray pd_tmp;
    KDL::JntArray pdd_tmp;
    KDL::JntArray rne_torques;
    KDL::Wrenches ext_force;
    Eigen::VectorXd Kp, Kd;
    Eigen::VectorXd Kp_cart, Kd_cart;

    //Khatib controller:
	Eigen::MatrixXd M_bar;
	Eigen::VectorXd C_bar, G_bar, CG_bar;
	Eigen::VectorXd Forces;

	KDL::Jacobian _jac, _jac_dot;
	KDL::JntArrayVel joint_position_velocity_des;

	KDL::JntArray q_from_robot;
	KDL::JntArray qd_from_robot;
	Eigen::VectorXd ref_acc;
	Eigen::MatrixXd tmpeye77;
	Eigen::MatrixXd tmpeye66;
	Eigen::MatrixXd preLambda;
	Eigen::VectorXd _ref_acc;
	Eigen::VectorXd _lambda_des;
	Eigen::MatrixXd P, P_tau;
	Eigen::MatrixXd N;

	Eigen::MatrixXd _inertia_cstr, _jac_cstr;
	Eigen::VectorXd _coriolis_cstr;

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
