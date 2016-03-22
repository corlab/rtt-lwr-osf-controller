#include "OrientationHelper.hpp"
//#include <rtt/RTT.hpp>

void OrientationHelper::Euler2Quaternion(Eigen::Vector3d & euler, double & quaternionV, Eigen::Vector3d & quaternionU) {
	// equals quaternion = log(euler)
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

void OrientationHelper::Quaternion2Euler(double & quaternionV, Eigen::Vector3d & quaternionU, Eigen::Vector3d & euler) {
	// equals euler = log(quaternion)
	if(quaternionU(0) == 0.0 && quaternionU(1) == 0.0 && quaternionU(2) == 0.0){
		euler.setZero();
	}
	else{
		euler = acos(quaternionV) * quaternionU / quaternionU.norm();

		//or use .squaredNorm() here??
//		euler = acos(quaternionV) * quaternionU / quaternionU.squaredNorm();
	}
}

void OrientationHelper::QuaternionProduct(double & quaternionV1, Eigen::Vector3d & quaternionU1, double & quaternionV2, Eigen::Vector3d & quaternionU2, double & resultV, Eigen::Vector3d & resultU) {
	resultV = 	quaternionV1 * quaternionV2 - quaternionU1.transpose() * quaternionU2;
	resultU = 	quaternionV1 * quaternionU2 +
				quaternionV2 * quaternionU1 +
				quaternionU1.cross(quaternionU2);
}


