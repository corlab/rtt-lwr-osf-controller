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

class RTTJointPositionController: public RTT::TaskContext {
public:
    RTTJointPositionController(std::string const& name, unsigned int numJoints);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    //output joint torque port
    RTT::OutputPort<rci::JointTorquesPtr> cmdJntTrq_Port;

    //input joint ports
    //RTT::InputPort<rci::JointAnglesPtr> currJntPos_Port;
    RTT::InputPort<Eigen::VectorXd> currJntPos_Port;
    RTT::FlowStatus currJntPos_Flow;

    //RTT::InputPort<rci::JointVelocitiesPtr> currJntVel_Port;
    RTT::InputPort<Eigen::VectorXd> currJntVel_Port;
    RTT::FlowStatus currJntVel_Flow;

    //RTT::InputPort<rci::JointAnglesPtr> refJntPos_Port;
    RTT::InputPort<Eigen::VectorXd> refJntPos_Port;
    RTT::FlowStatus refJntPos_Flow;

    //RTT::InputPort<rci::JointVelocitiesPtr> refJntVel_Port;
    RTT::InputPort<Eigen::VectorXd> refJntVel_Port;
    RTT::FlowStatus refJntVel_Flow;

protected:
    //output joint torque port
    rci::JointTorquesPtr cmdJntTrq_;
    Eigen::VectorXd cmdJntTrq;

    //rci::JointAnglesPtr currJntPositionFB;

    //input joint ports
    Eigen::VectorXd currJntPos;
    Eigen::VectorXd currJntVel;
    Eigen::VectorXd refJntPos;
    Eigen::VectorXd refJntVel;

    Eigen::VectorXd Kp, Kd;

    // parameters from config-file
    double Kp_StiffnessGain;
    double Kd_DampingGain;

    //other stuff
    unsigned int numJoints;

    //dynamic properties
    KDL::JntArray G_;

};
#endif
