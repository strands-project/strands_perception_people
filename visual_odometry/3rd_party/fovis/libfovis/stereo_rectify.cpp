#include "stereo_rectify.hpp"

#include <iostream>

namespace fovis
{

void
stereo_rectify(const CameraIntrinsicsParameters& left_params,
               const CameraIntrinsicsParameters& right_params,
               const Eigen::Quaterniond& rotation_quat,
               const Eigen::Vector3d& translation,
               Eigen::Matrix3d* left_rotation,
               Eigen::Matrix3d* right_rotation,
               CameraIntrinsicsParameters* rectified_params)
{
  // Get two cameras to same orientation, minimally rotating each.
  Eigen::AngleAxisd rect_rot(rotation_quat);
  rect_rot.angle() *= -0.5;

  Eigen::Vector3d rect_trans = rect_rot * translation;
  // Bring translation to alignment with (1, 0, 0).
  Eigen::Matrix3d rot;
  rot.row(0) = -rect_trans;
  rot.row(0).normalize();
  rot.row(1) = Eigen::Vector3d(0, 0, 1).cross(rot.row(0));
  rot.row(2) = rot.row(0).cross(rot.row(1));

  //std::cerr << "rot =\n" << rot << std::endl << std::endl;

  (*left_rotation) = rot * rect_rot.inverse();
  (*right_rotation) = rot * rect_rot;

  // For now just use left camera's intrinsic parameters
  *rectified_params = left_params;

  // Just to be explicit
  rectified_params->k1 = 0;
  rectified_params->k2 = 0;
  rectified_params->k3 = 0;
  rectified_params->p1 = 0;
  rectified_params->p2 = 0;
}

}
