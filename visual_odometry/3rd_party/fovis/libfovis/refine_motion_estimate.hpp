#ifndef __refine_motion_estimate_hpp__
#define __refine_motion_estimate_hpp__

#include <Eigen/Geometry>

namespace fovis
{

/**
 *
 * Given an initial motion estimate M_0, iteratively refines the motion
 * estimate to minimize reprojection error.
 *
 * M = argmin_{M} \sum_{i} || ref_projections[i] - p(K * M * points[i]) ||^2
 *
 * where K is the camera projection matrix formed by fx, cx, and cy:
 *
 * K = [ fx 0 cx 0
 *       0 fx cy 0
 *       0  0  1 0 ]
 *
 * and p(X) is the normalized form of a homogeneous coordinate.
 *
 */
Eigen::Isometry3d refineMotionEstimate(
    const Eigen::Matrix<double, 4, Eigen::Dynamic>& points,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_projections,
    double fx, double cx, double cy,
    const Eigen::Isometry3d& initial_estimate,
    int max_iterations);

/**
 *
 * Given an initial motion estimate M_0, iteratively refines the motion
 * estimate to minimize bidirectional reprojection error.
 *
 * M = argmin_{M} \sum_{i} ref_err(i) + target_err(i)
 *
 * ref_err(i) = || ref_projections[i] - p(K * M * target_points[i]) ||^2
 * target_err(i) = || target_projections[i] - p(K * M * ref_points[i]) ||^2
 *
 * where K is the camera projection matrix formed by fx, fy, cx, and cy:
 *
 * K = [ fx 0 cx 0
 *       0 fy cy 0
 *       0  0  1 0 ]
 *
 * and p(X) is the normalized form of a homogeneous coordinate.
 *
 */
void refineMotionEstimateBidirectional(
    const Eigen::Matrix<double, 4, Eigen::Dynamic>& ref_points,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_projections,
    const Eigen::Matrix<double, 4, Eigen::Dynamic>& target_points,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& target_projections,
    double fx, double cx, double cy,
    const Eigen::Isometry3d& initial_estimate,
    int max_iterations,
    Eigen::Isometry3d* result,
    Eigen::MatrixXd* result_covariance);

}

#endif
