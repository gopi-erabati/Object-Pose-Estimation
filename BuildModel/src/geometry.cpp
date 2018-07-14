#include "geometry.h"
using namespace cv;

void getOrientedVector(const Eigen::Quaternionf &q, cv::Mat_<float> &cvpt)
{
    double roll;
    double pitch;
    double yaw;
    // roll (x-axis rotation)
    double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny, cosy);


     cvpt= cv::Mat_<float>(3,1);
    cvpt(0,0) = 0;
    cvpt(1,0) = 0;
    cvpt(2,0) = -1;
    double nm = sqrt(cvpt(0,0)*cvpt(0,0) + cvpt(1,0)*cvpt(1,0) + cvpt(2,0)*cvpt(2,0));
    cvpt(0,0) /= nm;
    cvpt(1,0) /= nm;
    cvpt(2,0) /= nm;

    Mat_<float> gtR;

    Vec3f theta;
    theta[0] = roll;
    theta[1] = pitch;
    theta[2] = yaw;


    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );

    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );

    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);

//    gtR = R_y * R_x;
    gtR = R_z * R_y * R_x;

    cvpt = gtR*cvpt;
}



Eigen::Quaternionf euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitX());

    Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;
    return q;
}


void quaternion2EulerAngle(const Eigen::Quaternionf &q, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny, cosy);
}


void computeEulerAngularError(const Eigen::Quaternionf &q1,
                              const Eigen::Quaternionf &q2,
                              float &err_roll, float &err_pitch, float &err_yaw){
    double roll1,  roll2;
    double pitch1, pitch2;
    double yaw1,   yaw2;

    quaternion2EulerAngle(q1, roll1, pitch1, yaw1);
    quaternion2EulerAngle(q2, roll2, pitch2, yaw2);

    err_roll  = fabs(roll1-roll2);
    err_pitch = fabs(pitch1-pitch2);
    err_yaw = fabs(yaw1-yaw2);

}

cv::Mat eulerAnglesToRotationMatrix(Eigen::Vector3f &theta)
{
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<float>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]*M_PI/180),   -sin(theta[0]*M_PI/180),
               0,       sin(theta[0]*M_PI/180),   cos(theta[0]*M_PI/180)
               );

    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<float>(3,3) <<
               cos(theta[1]*M_PI/180),    0,      sin(theta[1]*M_PI/180),
               0,               1,      0,
               -sin(theta[1]*M_PI/180),   0,      cos(theta[1]*M_PI/180)
               );

    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<float>(3,3) <<
               cos(theta[2]*M_PI/180),    -sin(theta[2]*M_PI/180),      0,
               sin(theta[2]*M_PI/180),    cos(theta[2]*M_PI/180),       0,
               0,               0,                  1);


    // Combined rotation matrix
    cv::Mat R = R_z * R_y * R_x;

    return R;

}
