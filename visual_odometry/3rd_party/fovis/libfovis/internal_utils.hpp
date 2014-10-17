#ifndef __fovis_internal_utils_hpp__
#define __fovis_internal_utils_hpp__

#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <emmintrin.h>

#include "options.hpp"

namespace fovis
{

#define FOVIS_IS_ALIGNED16(x) (((uintptr_t)(x) & 0xf) == 0)

static inline int
round_up_to_multiple(int x, int a)
{
  int rem = x % a;
  if(rem)
    return x + (a - rem);
  return x;
}

static inline Eigen::Vector3d
_quat_to_roll_pitch_yaw(const Eigen::Quaterniond&q)
{
  double roll_a = 2 * (q.w() * q.x() + q.y() * q.z());
  double roll_b = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  double roll = atan2(roll_a, roll_b);

  double pitch_sin = 2 * (q.w() * q.y() - q.z() * q.x());
  double pitch = asin(pitch_sin);

  double yaw_a = 2 * (q.w() * q.z() + q.x() * q.y());
  double yaw_b = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  double yaw = atan2(yaw_a, yaw_b);
  return Eigen::Vector3d(roll, pitch, yaw);
}

static inline Eigen::Quaterniond
_rpy_to_quat(const Eigen::Vector3d rpy)
{
  double roll = rpy(0), pitch = rpy(1), yaw = rpy(2);

  double halfroll = roll / 2;
  double halfpitch = pitch / 2;
  double halfyaw = yaw / 2;

  double sin_r2 = sin(halfroll);
  double sin_p2 = sin(halfpitch);
  double sin_y2 = sin(halfyaw);

  double cos_r2 = cos(halfroll);
  double cos_p2 = cos(halfpitch);
  double cos_y2 = cos(halfyaw);

  Eigen::Quaterniond q;
  q.w() = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2;
  q.x() = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2;
  q.y() = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2;
  q.z() = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2;
  return q;
}

static inline void
print_isometry(const Eigen::Isometry3d & iso)
{
  const Eigen::Vector3d & t = iso.translation();
  Eigen::Vector3d rpy = _quat_to_roll_pitch_yaw(Eigen::Quaterniond(iso.rotation()))*180.0/M_PI;
  fprintf(stderr, "trans:(% 6.3f % 6.3f % 6.3f) rot:(% 6.3f % 6.3f % 6.3f)",t(0),t(1),t(2),rpy(0),rpy(1),rpy(2));
  //    dbg("rot:(% 6.3f % 6.3f % 6.3f)",rpy(0),rpy(1),rpy(2));
}

bool optionsGetInt(const VisualOdometryOptions& options, std::string name,
        int* result);

bool optionsGetBool(const VisualOdometryOptions& options, std::string name,
        bool* result);

int optionsGetIntOrFromDefault(const VisualOdometryOptions& options,
        std::string name, const VisualOdometryOptions& defaults);

bool optionsGetBoolOrFromDefault(const VisualOdometryOptions& options,
        std::string name, const VisualOdometryOptions& defaults);

bool optionsGetDouble(const VisualOdometryOptions& options, std::string name,
        double* result);

double optionsGetDoubleOrFromDefault(const VisualOdometryOptions& options,
        std::string name, const VisualOdometryOptions& defaults);

}

#endif
