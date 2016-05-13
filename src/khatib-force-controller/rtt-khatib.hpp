#ifndef RTT_JPC_HPP
#define RTT_JPC_HPP
// RTT includes
#include <rtt/RTT.hpp>
#include <rtt/base/RunnableInterface.hpp>
#include <rtt/Activity.hpp>
// RCI includes
#include <rci/dto/JointAngles.h>
#include <rci/dto/JointVelocities.h>
#include <rci/dto/JointTorques.h>
#include <nemo/Vector.h>
#include <nemo/Mapping.h>
// KDL includes
#include <kdl/tree.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainfksolver.hpp>

// BOOST includes
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
// Parser include convert URDF/SDF into KDL::Chain
//#include "parsertools/KDLParser.hpp"

class RTTKhatibForceController: public RTT::TaskContext {
public:
    RTTKhatibForceController(std::string const& name, unsigned int numJoints);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    //output joint torque port
    RTT::OutputPort<rci::JointTorquesPtr> cmdJntTrq_Port;

    //input joint ports
    RTT::InputPort<Eigen::VectorXd> currJntPos_Port;
    RTT::FlowStatus currJntPos_Flow;

    RTT::InputPort<Eigen::VectorXd> currJntVel_Port;
    RTT::FlowStatus currJntVel_Flow;

    //input cartesian translation ports
    RTT::InputPort<Eigen::Vector3d> currTaskEEPosTranslation_Port;
    RTT::FlowStatus currTaskEEPosTranslation_Flow;

    RTT::InputPort<Eigen::Vector3d> currTaskEEVelTranslation_Port;
	RTT::FlowStatus currTaskEEVelTranslation_Flow;

    RTT::InputPort<Eigen::Vector3d> refTaskEEPosTranslation_Port;
    RTT::FlowStatus refTaskEEPosTranslation_Flow;

    RTT::InputPort<Eigen::Vector3d> refTaskEEVelTranslation_Port;
	RTT::FlowStatus refTaskEEVelTranslation_Flow;

	RTT::InputPort<Eigen::Vector3d> refTaskEEAccTranslation_Port;
	RTT::FlowStatus refTaskEEAccTranslation_Flow;

    //input cartesian orientation ports
    RTT::InputPort<Eigen::Vector3d> currTaskEEPosOrientation_Port;
    RTT::FlowStatus currTaskEEPosOrientation_Flow;

    RTT::InputPort<Eigen::Vector3d> currTaskEEVelOrientation_Port;
	RTT::FlowStatus currTaskEEVelOrientation_Flow;

    RTT::InputPort<Eigen::Vector3d> refTaskEEPosOrientation_Port;
    RTT::FlowStatus refTaskEEPosOrientation_Flow;

    RTT::InputPort<Eigen::Vector3d> refTaskEEVelOrientation_Port;
	RTT::FlowStatus refTaskEEVelOrientation_Flow;

	RTT::InputPort<Eigen::Vector3d> refTaskEEAccOrientation_Port;
	RTT::FlowStatus refTaskEEAccOrientation_Flow;


private:
	//output joint torque port
    rci::JointTorquesPtr cmdJntTrq;

    //input joint ports
    Eigen::VectorXd currJntPos;
    Eigen::VectorXd currJntVel;

    //input cartesian translation ports
    Eigen::Vector3d currTaskEEPosTranslation;
    Eigen::Vector3d currTaskEEVelTranslation;
    Eigen::Vector3d refTaskEEPosTranslation;
    Eigen::Vector3d refTaskEEVelTranslation;
    Eigen::Vector3d refTaskEEAccTranslation;

    //input cartesian orientation ports
    Eigen::Vector3d currTaskEEPosOrientation;
    Eigen::Vector3d currTaskEEVelOrientation;
    Eigen::Vector3d refTaskEEPosOrientation;
    Eigen::Vector3d refTaskEEVelOrientation;
    Eigen::Vector3d refTaskEEAccOrientation;

    // parameters from config-file
    bool use_original_khatib_controller;
    bool use_euler_orientation;

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

    //other stuff
    unsigned int numJoints;
    Eigen::VectorXd Kp_cartTranslation, Kd_cartTranslation;
	Eigen::VectorXd Kp_cartOrientationEuler, Kd_cartOrientationEuler;
	Eigen::VectorXd Kp_cartOrientationQuaternion, Kd_cartOrientationQuaternion;
	Eigen::VectorXd Kp_joint, Kd_joint;
	Eigen::MatrixXd identity77, identity66, identity33;
	Eigen::MatrixXd diagonal66, diagonal33;
	Eigen::MatrixXd tmpeye77, tmpeye66;

    Eigen::Vector3d ref_accTranslation, ref_accOrientation;
    Eigen::VectorXd ref_acc;
	Eigen::MatrixXd Lamda, Lamda_cstr;
	Eigen::VectorXd CG_bar;
	Eigen::VectorXd h;
	Eigen::VectorXd Forces, Forces_cstr;
	Eigen::VectorXd tau_0;
	Eigen::MatrixXd N;
	Eigen::MatrixXd P;
	Eigen::MatrixXd jac_cstr_, jac_cstr_MPI;
	Eigen::MatrixXd M_cstr_;
	Eigen::MatrixXd C_cstr_;
	Eigen::VectorXd lambda_des;

	Eigen::VectorXd jnt_trq_cmd_Motion_Khatib, jnt_trq_cmd_Nullspace_Khatib;
	Eigen::VectorXd jnt_trq_cmd_Motion_Projected, jnt_trq_cmd_Nullspace_Projected, jnt_trq_cmd_Force_Projected;

	KDL::JntSpaceInertiaMatrix M_;
	KDL::JntArray C_;
	KDL::JntArray G_;
	KDL::Jacobian jac_;
	KDL::Jacobian jac_dot_;

    KDL::JntArray q_des_Nullspace;
    KDL::JntArray q_des_FirstPoint;

//	//start variables for quaternion feedback stuff
//	rci::OrientationPtr desiredCartOrientation;
//	rci::OrientationPtr currCartOrientation;
//	double desiredCartOrientationQuaternionV; //could be also double
//	Eigen::Vector3d desiredCartOrientationQuaternionU;
//	double currCartOrientationQuaternionV; //could be also double
//	Eigen::Vector3d currCartOrientationQuaternionU;
//	double QuaternionProductV; //could be also double
//	Eigen::Vector3d QuaternionProductU;
//	Eigen::Vector3d QuaternionProductEuler;
//	//stop variables for quaternion feedback stuff
};
#endif
