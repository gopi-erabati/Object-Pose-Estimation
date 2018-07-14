#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>



void getOrientedVector(const Eigen::Quaternionf& q, cv::Mat_<float> &cvpt);

Eigen::Quaternionf euler2Quaternion(const double roll, const double pitch, const double yaw);

void quaternion2EulerAngle(const Eigen::Quaternionf& q, double& roll, double& pitch, double& yaw);

void computeEulerAngularError(const Eigen::Quaternionf& q1,
                              const Eigen::Quaternionf& q2,
                              float &err_roll, float &err_pitch, float &err_yaw);

// Calculates rotation matrix given euler angles.
cv::Mat eulerAnglesToRotationMatrix(Eigen::Vector3f &theta);



#endif // GEOMETRY_H
