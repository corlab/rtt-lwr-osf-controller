#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class OrientationHelper {
public:
	OrientationHelper() { };
	~OrientationHelper() { };

	void Euler2Quaternion(Eigen::Vector3d & euler, double & quaternionV, Eigen::Vector3d & quaternionU);

	void Quaternion2Euler(double & quaternionV, Eigen::Vector3d & quaternionU, Eigen::Vector3d & euler);

	void QuaternionProduct(double & quaternionV1, Eigen::Vector3d & quaternionU1, double & quaternionV2, Eigen::Vector3d & quaternionU2, double & resultV, Eigen::Vector3d & resultU);

};

