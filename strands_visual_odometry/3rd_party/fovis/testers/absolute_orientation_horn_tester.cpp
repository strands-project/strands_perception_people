#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Core>

#include "../libfovis/absolute_orientation_horn.hpp"

using namespace std;

static int
rand_int_range(int min, int max)
{
  return rand() % (max - min) + min;
}

static double 
rand_double_range(double min, double max)
{
  double v = rand() / (double)RAND_MAX;
  return v * (max - min) + min;
}

void 
rpy_to_quat (const double rpy[3], double q[4])
{
  double roll = rpy[0], pitch = rpy[1], yaw = rpy[2];

  double halfroll = roll / 2;
  double halfpitch = pitch / 2;
  double halfyaw = yaw / 2;

  double sin_r2 = sin (halfroll);
  double sin_p2 = sin (halfpitch);
  double sin_y2 = sin (halfyaw);

  double cos_r2 = cos (halfroll);
  double cos_p2 = cos (halfpitch);
  double cos_y2 = cos (halfyaw);

  q[0] = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2;
  q[1] = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2;
  q[2] = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2;
  q[3] = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2;
}

int main(int argc, char** argv)
{
  int num_trials = 1000;

  for(int trial=0; trial<num_trials; trial++) {

    // generate a random set of points
    int num_points = rand_int_range(5, 1000);

    Eigen::Matrix3Xd points(3, num_points);

    for(int col=0; col<num_points; col++) {
      points(0, col) = rand_double_range(-10, 10);
      points(1, col) = rand_double_range(-10, 10);
      points(2, col) = rand_double_range(-10, 10);
    }

    // generate a random transformation
    Eigen::Vector3d translation(rand_double_range(-10, 10),
        rand_double_range(-10, 10),
        rand_double_range(-10, 10));

    double rpy[3] = {
      rand_double_range(-M_PI, M_PI),
      rand_double_range(-M_PI, M_PI),
      rand_double_range(-M_PI, M_PI) };
    double rot_quat[4];
    rpy_to_quat(rpy, rot_quat);

    Eigen::Quaterniond rotation(rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3]);

    Eigen::Isometry3d trans;
    trans.setIdentity();
    trans.translate(translation);
    trans.rotate(rotation);

    // apply transformation to original random point set
    Eigen::Matrix3Xd transformed = trans * points;

    Eigen::Isometry3d estimated_transform;
    absolute_orientation_horn(points, transformed, &estimated_transform);

    // reproject points using estimated transformation
    Eigen::Matrix3Xd reprojected = estimated_transform * points;

    // compute reprojection error
    Eigen::Matrix3Xd reproject_err = reprojected - transformed;

    Eigen::Vector3d mean_err = reproject_err.rowwise().sum() / num_points;
    printf("%4d (%4d): mean reprojection error: %6.3f, %6.3f, %6.3f\n", trial, num_points, mean_err(0), mean_err(1), mean_err(2));
    if(fabs(mean_err(0)) > 1e-9 || fabs(mean_err(1)) > 1e-9 || fabs(mean_err(1)) > 1e-9) {
      fprintf(stderr, "FAIL!\n");
      exit(1);
    }
  }

  fprintf(stderr, "OK\n");
  return 0;
}
