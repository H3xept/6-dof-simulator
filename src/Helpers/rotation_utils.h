#ifndef __ROTATION_UTILS_H__
#define __ROTATION_UTILS_H__

#include <Eigen/Eigen>
#include <cmath>

inline Eigen::VectorXd euler_angles_to_quaternions(const Eigen::Vector3d euler_rpy) {
        Eigen::VectorXd quaternion{4};
        float roll = euler_rpy[0];
        float pitch = euler_rpy[1];
        float yaw = euler_rpy[2];
        // Order should be w x y z 
        quaternion[1] = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
        quaternion[2] = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
        quaternion[3] = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
        quaternion[0] = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
        return quaternion;
}
    
#endif // __ROTATION_UTILS_H__