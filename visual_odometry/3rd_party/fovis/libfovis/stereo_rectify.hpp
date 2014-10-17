#ifndef __fovis_stereo_rectify_hpp__
#define __fovis_stereo_rectify_hpp__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "camera_intrinsics.hpp"

namespace fovis
{

void
stereo_rectify(const CameraIntrinsicsParameters& left_params,
               const CameraIntrinsicsParameters& right_params,
               const Eigen::Quaterniond& rotation_quat,
               const Eigen::Vector3d& translation,
               Eigen::Matrix3d* left_rotation,
               Eigen::Matrix3d* right_rotation,
               CameraIntrinsicsParameters* rectified_params);
}

#endif
