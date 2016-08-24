/* Author: Milad Malekzadeh
 * Date:   16 August 2016
 *
 * Description:
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

//#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
//#include <rst-rt/kinematics/JointAngles.hpp>
 #include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/geometry/Rotation.hpp>
#include <rst-rt/geometry/Translation.hpp>

class PositionController: public RTT::TaskContext {
public:
	PositionController(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsize(unsigned int DOFsize);
    void setGains(float kp, float kd);
    void setGainsOrientation(float kp, float kd);
    void preparePorts();
    void displayStatus();
    void toEulerAngles(Eigen::Vector3f& res, Eigen::Quaternionf const & quat) const;
    void toQuaternion(Eigen::Vector3f const & rpy, Eigen::Quaternionf& res) const;

    void QuaternionProduct(
            Eigen::Quaternionf const & quat1,
            Eigen::Quaternionf const & quat2,
            Eigen::Quaternionf& quatResult);
    void QuaternionProduct(
            float const & quaternionV1,
            Eigen::Vector3f const & quaternionU1,
            float const & quaternionV2,
            Eigen::Vector3f const & quaternionU2,
            float & resultV,
            Eigen::Vector3f & resultU);

    void ExpEuler2Quaternion(Eigen::Vector3f const & euler, Eigen::Quaternionf & quaternion);
    void ExpEuler2Quaternion(Eigen::Vector3f const & euler, float & quaternionV, Eigen::Vector3f & quaternionU);
    void LogQuaternion2Euler(Eigen::Quaternionf const & quaternion, Eigen::Vector3f & euler);
    void LogQuaternion2Euler(float const & quaternionV, Eigen::Vector3f const & quaternionU, Eigen::Vector3f & euler);

private:
    // Declare input ports and their datatypes
    RTT::InputPort<Eigen::VectorXf> in_desiredTaskSpacePosition_port;
    RTT::InputPort<Eigen::VectorXf> in_desiredTaskSpaceVelocity_port;
    RTT::InputPort<Eigen::VectorXf> in_desiredTaskSpaceAcceleration_port;

    RTT::InputPort<Eigen::VectorXf> in_currentTaskSpacePosition_port;
    RTT::InputPort<Eigen::VectorXf> in_currentTaskSpaceVelocity_port;
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;

    RTT::InputPort<Eigen::MatrixXf> in_jacobian_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianDot_port;
    RTT::InputPort<Eigen::MatrixXf> in_constraintLambda_port;
    RTT::InputPort<Eigen::VectorXf> in_h_port;
    RTT::InputPort<Eigen::MatrixXf> in_constraintM_port;
    RTT::InputPort<Eigen::MatrixXf> in_P_port;
    RTT::InputPort<Eigen::MatrixXf> in_constraintC_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;

    // Data flow:
    RTT::FlowStatus in_desiredTaskSpacePosition_flow;
    RTT::FlowStatus in_desiredTaskSpaceVelocity_flow;
    RTT::FlowStatus in_desiredTaskSpaceAcceleration_flow;

    RTT::FlowStatus in_currentTaskSpacePosition_flow;
    RTT::FlowStatus in_currentTaskSpaceVelocity_flow;
    RTT::FlowStatus in_robotstatus_flow;

    RTT::FlowStatus in_jacobian_flow;
    RTT::FlowStatus in_jacobianDot_flow;
    RTT::FlowStatus in_constraintLambda_flow;
    RTT::FlowStatus in_h_flow;
    RTT::FlowStatus in_constraintM_flow;
    RTT::FlowStatus in_P_flow;
    RTT::FlowStatus in_constraintC_flow;

    // variables
    Eigen::VectorXf in_desiredTaskSpacePosition_var;
    Eigen::VectorXf in_desiredTaskSpaceVelocity_var;
    Eigen::VectorXf in_desiredTaskSpaceAcceleration_var;

    Eigen::VectorXf in_currentTaskSpacePosition_var;
    Eigen::VectorXf in_currentTaskSpaceVelocity_var;
    rstrt::robot::JointState in_robotstatus_var;

    Eigen::MatrixXf in_jacobian_var;
    Eigen::MatrixXf in_jacobianDot_var;
    Eigen::MatrixXf in_constraintLambda_var;
    Eigen::VectorXf in_h_var;
    Eigen::MatrixXf in_constraintM_var;
    Eigen::MatrixXf in_P_var;
    Eigen::MatrixXf in_constraintC_var;

    rstrt::dynamics::JointTorques out_torques_var;

    Eigen::AngleAxisf rotx, roty, rotz;
    Eigen::Quaternionf quat_target,quat_current,quat_diff;
    Eigen::Vector3f euler_diff_pos, euler_diff_vel, euler_temp,euler_temp2;
    Eigen::Matrix3f temp_mat;
    Eigen::VectorXf error_pos, error_vel;

    unsigned int DOFsize;
    bool receiveTranslationOnly;
    unsigned int TaskSpaceDimension;
    float gainP, gainD, gainP_o, gainD_o;
    bool portsArePrepared;
    Eigen::VectorXf ref_Acceleration, constraintForce;

    // call before preparePorts()
	void setTranslationOnly(const bool translationOnly);
};

