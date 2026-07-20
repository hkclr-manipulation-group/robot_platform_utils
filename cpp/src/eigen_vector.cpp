#include "eigen_vector.h"
#include <Eigen/Geometry>

namespace robot::platform::eigen {
    Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
        // 1. Create individual rotation objects around principal axes
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        // 2. Combine them in the standard ZYX (yaw, pitch, roll) order
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        
        return q; // Returns normalized quaternion
    }

    Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) {
        // 1. Convert quaternion to rotation matrix
        Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();
        
        // 2. Extract angles using ZYX sequence (2 = Z, 1 = Y, 0 = X)
        // The returned vector elements match the order of execution: [Z, Y, X]
        Eigen::Vector3d euler = rotationMatrix.eulerAngles(2, 1, 0);
        
        // euler[0] = Yaw (Z)
        // euler[1] = Pitch (Y)
        // euler[2] = Roll (X)
        return euler;
    }
}  // namespace robot::platform::eigen
