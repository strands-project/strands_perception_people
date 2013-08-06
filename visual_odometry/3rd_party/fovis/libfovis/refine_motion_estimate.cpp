#include <stdio.h>
#include <iostream>
#include <assert.h>

#include "refine_motion_estimate.hpp"

#define dump(v) std::cerr << #v << " : " << (v) << "\n"
#define dumpT(v) std::cerr << #v << " : " << (v).transpose() << "\n"

//#define dbg(...) fprintf(stderr, __VA_ARGS__)
#define dbg(...)

#define USE_ESM

namespace fovis
{

static inline Eigen::Isometry3d
isometryFromParams(const Eigen::Matrix<double, 6, 1>& params)
{
  Eigen::Isometry3d result;

  double roll = params(3), pitch = params(4), yaw = params(5);
  double halfroll = roll / 2;
  double halfpitch = pitch / 2;
  double halfyaw = yaw / 2;
  double sin_r2 = sin(halfroll);
  double sin_p2 = sin(halfpitch);
  double sin_y2 = sin(halfyaw);
  double cos_r2 = cos(halfroll);
  double cos_p2 = cos(halfpitch);
  double cos_y2 = cos(halfyaw);

  Eigen::Quaterniond quat(
    cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2,
    sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2,
    cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2,
    cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2);

  result.setIdentity();
  result.translate(params.head<3>());
  result.rotate(quat);

  return result;
}

static inline Eigen::Matrix<double, 6, 1>
isometryToParams(const Eigen::Isometry3d& M)
{
  Eigen::Quaterniond q(M.rotation());
  double roll_a = 2 * (q.w()*q.x() + q.y()*q.z());
  double roll_b = 1 - 2 * (q.x()*q.x() + q.y()*q.y());
  double pitch_sin = 2 * (q.w()*q.y() - q.z()*q.x());
  double yaw_a = 2 * (q.w()*q.z() + q.x()*q.y());
  double yaw_b = 1 - 2 * (q.y()*q.y() + q.z()*q.z());

  Eigen::Matrix<double, 6, 1> result;
  result.head<3>() = M.translation();
  result(3) = atan2(roll_a, roll_b);
  result(4) = asin(pitch_sin);
  result(5) = atan2(yaw_a, yaw_b);
  return result;
}

static void
computeReprojectionError(const Eigen::Matrix<double, 4, Eigen::Dynamic>& points,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_projections,
    const Eigen::Matrix<double, 3, 4>& K,
    const Eigen::Isometry3d& motion,
    Eigen::VectorXd* err,
    int err_offset)
{
  int num_points = points.cols();
  assert(((err->size() == num_points * 2) && (err_offset == 0)) || 
         ((err->size() == num_points * 4) && ((err_offset == 0) || (err_offset == num_points*2))));
  assert(num_points == ref_projections.cols());

  Eigen::Matrix<double, 3, 4> P = K * motion.matrix();

  for(int pind=0; pind < num_points; pind++) {
    Eigen::Vector3d uvw = P * points.col(pind);
    double u = uvw(0) / uvw(2);
    double v = uvw(1) / uvw(2);
    (*err)(err_offset + pind*2 + 0) = u - ref_projections(0, pind);
    (*err)(err_offset + pind*2 + 1) = v - ref_projections(1, pind);
  }
}

static void
computeProjectionJacobian(const Eigen::Matrix<double, 6, 1>& params,
    double fx, double px, double py,
    const Eigen::Matrix<double, 4, Eigen::Dynamic>& points,
    Eigen::Matrix<double, Eigen::Dynamic, 6> *result,
    int result_row_offset)
{
  double tx    = params(0);
  double ty    = params(1);
  double tz    = params(2);
  double roll  = params(3);
  double pitch = params(4);
  double yaw   = params(5);
  double sr   = sin(roll);
  double cr   = cos(roll);
  double sp   = sin(pitch);
  double cp   = cos(pitch);
  double sy   = sin(yaw);
  double cy   = cos(yaw);

  // projection matrix
  Eigen::Matrix<double, 3, 4> P;
  P << 
    fx*cp*cy - px*sp, px*cp*sr - fx*(cr*sy - cy*sp*sr), fx*(sr*sy + cr*cy*sp) + px*cp*cr, fx*tx + px*tz,
    fx*cp*sy - py*sp, fx*(cr*cy + sp*sr*sy) + py*cp*sr, py*cp*cr - fx*(cy*sr - cr*sp*sy), fx*ty + py*tz,
    -sp, cp*sr, cp*cr, tz;

  // Jacobian matrices for homogeneous coordinates of point projections
  // thank you matlab
  Eigen::Matrix<double, 6, 4> Ju, Jv, Jw;
  Ju << 0, 0, 0, fx,
     0, 0, 0, 0,
     0, 0, 0, px,
     0, fx*(sr*sy + cr*cy*sp) + px*cp*cr,   fx*(cr*sy - cy*sp*sr) - px*cp*sr, 0,
     - px*cp - fx*cy*sp, -sr*(px*sp - fx*cp*cy), -cr*(px*sp - fx*cp*cy), 0,
     -fx*cp*sy, - fx*cr*cy - fx*sp*sr*sy, fx*cy*sr - fx*cr*sp*sy, 0;
  Jv << 0, 0, 0, 0,
     0, 0, 0, fx,
     0, 0, 0, py,
     0, py*cp*cr - fx*(cy*sr - cr*sp*sy), - fx*(cr*cy + sp*sr*sy) - py*cp*sr, 0,
     - py*cp - fx*sp*sy, -sr*(py*sp - fx*cp*sy), -cr*(py*sp - fx*cp*sy), 0,
     fx*cp*cy,   fx*cy*sp*sr - fx*cr*sy, fx*sr*sy + fx*cr*cy*sp, 0;
  Jw << 0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 1,
     0, cp*cr, -cp*sr, 0,
     -cp, -sp*sr, -cr*sp, 0,
     0, 0, 0, 0;

  int num_points = points.cols();
//  Eigen::Matrix<double, Eigen::Dynamic, 6> result(num_points*2, 6);
  assert(result->rows() == num_points*4 || result->rows() == num_points*2);
  assert(result->cols() == 6);
  assert(result_row_offset == 0);
  int row = 0;
  for(int i=0; i<num_points; i++) {
    const Eigen::Vector4d& point = points.col(i);
    Eigen::Vector3d uvw = P * point;
    Eigen::Matrix<double, 6, 1> du = Ju * point;
    Eigen::Matrix<double, 6, 1> dv = Jv * point;
    Eigen::Matrix<double, 6, 1> dw = Jw * point;

    double w_inv_sq = 1 / (uvw(2) * uvw(2));

    result->row(result_row_offset + row) = (du * uvw(2) - dw * uvw(0)) * w_inv_sq;
    result->row(result_row_offset + row+1) = (dv * uvw(2) - dw * uvw(1)) * w_inv_sq;

    row += 2;
  }
}

Eigen::Isometry3d 
refineMotionEstimate(const Eigen::Matrix<double, 4, Eigen::Dynamic>& points, 
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_projections,
        double fx, double px, double py,
        const Eigen::Isometry3d& initial_estimate,
        int max_iterations)
{
  Eigen::Matrix<double, 3, 4> xyz_c_to_uvw_c;
  xyz_c_to_uvw_c << fx, 0, px, 0,
                 0, fx, py, 0,
                 0, 0, 1, 0;

  Eigen::Isometry3d estimate = initial_estimate;
  Eigen::Matrix<double, 6, 1> estimate_vec = isometryToParams(estimate);

  int num_points = points.cols();

  // compute initial reprojection error
  Eigen::VectorXd err(num_points*2);
  computeReprojectionError(points, ref_projections, xyz_c_to_uvw_c, estimate, 
      &err, 0);

  double initial_sse = err.cwiseProduct(err).sum();
  double final_sse = initial_sse;
  dbg("T0 : [%7.3f %7.3f %7.3f] R: [%7.3f %7.3f %7.3f] E: %f\n", 
      estimate_vec(0), estimate_vec(1), estimate_vec(2), 
      estimate_vec(3), estimate_vec(4), estimate_vec(5), final_sse);

//  // TODO use a non-zero lambda value for actual levenberg-marquadt refinement
//  double lambda = 0;

  Eigen::Matrix<double, Eigen::Dynamic, 6> M(num_points*2, 6);

  for(int iter_num=0; iter_num<max_iterations; iter_num++) {
    computeProjectionJacobian(estimate_vec, fx, px, py, points, &M, 0);

    // Gauss-Newton:
    //    delta = -(pseudoinverse of M) * error
    //          = - inv(M'*M) * M' * error
    // Levenberg-Marquadt
    //    delta = - inv(M'*M + lambda * diag(M'*M)) * M' * error

    Eigen::Matrix<double, 6, 1> g = M.transpose() * err;
    Eigen::Matrix<double, 6, 6> MtM = M.transpose() * M;
//    Eigen::Matrix<double, 6, 6> diag(MtM.diagonal().asDiagonal());
//    MtM += lambda * diag;
    Eigen::Matrix<double, 6, 1> delta = - MtM.inverse() * g;

    // compute new isometry estimate
    Eigen::Matrix<double, 6, 1> next_params = estimate_vec + delta;
    Eigen::Isometry3d next_estimate = isometryFromParams(next_params);

    // compute reprojection error at new estimate
    computeReprojectionError(points, ref_projections, xyz_c_to_uvw_c, next_estimate, 
        &err, 0);
    double sse = err.cwiseProduct(err).sum();
    //std::cerr << iter_num << ": " << next_params.transpose() << " : " << sse << std::endl;

    // if stepping would increase the error, then just give up.
    if(sse > final_sse)
      break;

    // update estimate parameters and error values
    estimate_vec = next_params;
    estimate = next_estimate;
    final_sse = sse;

    // stop if we're not moving that much
    if(fabs(delta(0)) < 0.0001 &&
       fabs(delta(1)) < 0.0001 &&
       fabs(delta(2)) < 0.0001 &&
       fabs(delta(3)) < (0.01 * M_PI/180) &&
       fabs(delta(4)) < (0.01 * M_PI/180) &&
       fabs(delta(5)) < (0.01 * M_PI/180))
        break;

    dbg("T%-2d: [%7.3f %7.3f %7.3f] R: [%7.3f %7.3f %7.3f] E: %f\n", 
        iter_num+1,
        estimate_vec(0), estimate_vec(1), estimate_vec(2), 
        estimate_vec(3), estimate_vec(4), estimate_vec(5), final_sse);
  }
  dbg("T--: [%7.3f %7.3f %7.3f] R: [%7.3f %7.3f %7.3f] E: %f\n", 
      estimate_vec(0), estimate_vec(1), estimate_vec(2), 
      estimate_vec(3), estimate_vec(4), estimate_vec(5), final_sse);

  return estimate;
}

// ============= bidirectional reprojection error minimization ============

static void
computeReverseProjectionJacobian(const Eigen::Matrix<double, 6, 1>& params,
    double fx, double px, double py,
    const Eigen::Matrix<double, 4, Eigen::Dynamic>& points,
    Eigen::Matrix<double, Eigen::Dynamic, 6>* result,
    int result_row_offset)
{
  double tx    = params(0);
  double ty    = params(1);
  double tz    = params(2);
  double roll  = params(3);
  double pitch = params(4);
  double yaw   = params(5);
  double sr   = sin(roll);
  double cr   = cos(roll);
  double sp   = sin(pitch);
  double cp   = cos(pitch);
  double sy   = sin(yaw);
  double cy   = cos(yaw);

  // Reverse projection matrix.  Thank you matlab.
  Eigen::Matrix<double, 3, 4> P;
  P << px*(sr*sy + cr*cy*sp) + fx*cp*cy, 
       fx*cp*sy - px*(cy*sr - cr*sp*sy), 
       px*cp*cr - fx*sp, 
       - px*(tx*(sr*sy + cr*cy*sp) - ty*(cy*sr - cr*sp*sy) + tz*cp*cr) - fx*(tx*cp*cy - tz*sp + ty*cp*sy),

       py*(sr*sy + cr*cy*sp) - fx*(cr*sy - cy*sp*sr), 
       fx*(cr*cy + sp*sr*sy) - py*(cy*sr - cr*sp*sy), 
       cp*(py*cr + fx*sr), 
       - py*(tx*(sr*sy + cr*cy*sp) - ty*(cy*sr - cr*sp*sy) + tz*cp*cr) - fx*(ty*(cr*cy + sp*sr*sy) - tx*(cr*sy - cy*sp*sr) + tz*cp*sr),

       sr*sy + cr*cy*sp, 
       cr*sp*sy - cy*sr, 
       cp*cr, 
       ty*(cy*sr - cr*sp*sy) - tx*(sr*sy + cr*cy*sp) - tz*cp*cr;

  // Jacobian matrices for homogeneous coordinates of reverse point projections
  // thank you matlab
  Eigen::Matrix<double, 6, 4> Ju, Jv, Jw;
  Ju << 0, 0, 0, - px*(sr*sy + cr*cy*sp) - fx*cp*cy,
        0, 0, 0, px*(cy*sr - cr*sp*sy) - fx*cp*sy,
        0, 0, 0, fx*sp - px*cp*cr,
        px*cr*sy - px*cy*sp*sr, - px*cr*cy - px*sp*sr*sy, -px*cp*sr, px*ty*(cr*cy + sp*sr*sy) - px*tx*(cr*sy - cy*sp*sr) + px*tz*cp*sr,
        -cy*(fx*sp - px*cp*cr), -sy*(fx*sp - px*cp*cr), - fx*cp - px*cr*sp, fx*(tz*cp + tx*cy*sp + ty*sp*sy) - px*(tx*cp*cr*cy - tz*cr*sp + ty*cp*cr*sy),
        px*(cy*sr - cr*sp*sy) - fx*cp*sy, px*(sr*sy + cr*cy*sp) + fx*cp*cy, 0, - fx*(ty*cp*cy - tx*cp*sy) - px*(tx*(cy*sr - cr*sp*sy) + ty*(sr*sy + cr*cy*sp));

 Jv << 0, 0, 0, fx*(cr*sy - cy*sp*sr) - py*(sr*sy + cr*cy*sp),
       0, 0, 0, py*(cy*sr - cr*sp*sy) - fx*(cr*cy + sp*sr*sy),
       0, 0, 0, -cp*(py*cr + fx*sr),
       fx*(sr*sy + cr*cy*sp) + py*(cr*sy - cy*sp*sr), - fx*(cy*sr - cr*sp*sy) - py*(cr*cy + sp*sr*sy), cp*(fx*cr - py*sr), py*(ty*(cr*cy + sp*sr*sy) - tx*(cr*sy - cy*sp*sr) + tz*cp*sr) - fx*(tx*(sr*sy + cr*cy*sp) - ty*(cy*sr - cr*sp*sy) + tz*cp*cr),
       cp*cy*(py*cr + fx*sr), cp*sy*(py*cr + fx*sr), -sp*(py*cr + fx*sr), -(py*cr + fx*sr)*(tx*cp*cy - tz*sp + ty*cp*sy),
       py*(cy*sr - cr*sp*sy) - fx*(cr*cy + sp*sr*sy), py*(sr*sy + cr*cy*sp) - fx*(cr*sy - cy*sp*sr), 0, fx*(tx*(cr*cy + sp*sr*sy) + ty*(cr*sy - cy*sp*sr)) - py*(tx*(cy*sr - cr*sp*sy) + ty*(sr*sy + cr*cy*sp));

  Jw << 0, 0, 0, - sr*sy - cr*cy*sp,
        0, 0, 0, cy*sr - cr*sp*sy,
        0, 0, 0, -cp*cr,
        cr*sy - cy*sp*sr, - cr*cy - sp*sr*sy, -cp*sr, ty*(cr*cy + sp*sr*sy) - tx*(cr*sy - cy*sp*sr) + tz*cp*sr,
        cp*cr*cy, cp*cr*sy, -cr*sp, -cr*(tx*cp*cy - tz*sp + ty*cp*sy),
        cy*sr - cr*sp*sy, sr*sy + cr*cy*sp, 0, - tx*(cy*sr - cr*sp*sy) - ty*(sr*sy + cr*cy*sp);
       
  int num_points = points.cols();
  assert(result->rows() == num_points*4 || result->rows() == num_points*2);
  assert(result->cols() == 6);
  assert(result_row_offset == 0 || result_row_offset == (result->rows() / 2));
//  Eigen::Matrix<double, Eigen::Dynamic, 6> result(num_points*2, 6);
  int row = 0;
  for(int i=0; i<num_points; i++) {
    const Eigen::Vector4d& point = points.col(i);
    Eigen::Vector3d uvw = P * point;
    Eigen::Matrix<double, 6, 1> du = Ju * point;
    Eigen::Matrix<double, 6, 1> dv = Jv * point;
    Eigen::Matrix<double, 6, 1> dw = Jw * point;

    double w_inv_sq = 1 / (uvw(2) * uvw(2));

    result->row(result_row_offset + row) = (du * uvw(2) - dw * uvw(0)) * w_inv_sq;
    result->row(result_row_offset + row+1) = (dv * uvw(2) - dw * uvw(1)) * w_inv_sq;

    row += 2;
  }
}

void refineMotionEstimateBidirectional(const Eigen::Matrix<double, 4, Eigen::Dynamic>& ref_points, 
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_projections,
        const Eigen::Matrix<double, 4, Eigen::Dynamic>& target_points, 
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& target_projections,
        double fx, 
        double px, double py,
        const Eigen::Isometry3d& initial_estimate,
        int max_iterations,
        Eigen::Isometry3d* result,
        Eigen::MatrixXd* result_covariance)
{
  Eigen::Matrix<double, 3, 4> xyz_c_to_uvw_c;
  xyz_c_to_uvw_c << fx, 0, px, 0,
                 0, fx, py, 0,
                 0, 0, 1, 0;

  Eigen::Matrix<double, 6, 1> estimate_vec = isometryToParams(initial_estimate);

  int num_points = target_points.cols();

  // allocate space for reprojection error vector
  Eigen::VectorXd err(num_points*4);
  // reprojection error from target point cloud to reference image
  computeReprojectionError(target_points, ref_projections, xyz_c_to_uvw_c, 
      initial_estimate, &err, 0);
  // reprojection error from reference point cloud to target image
  computeReprojectionError(ref_points, target_projections, xyz_c_to_uvw_c, 
      initial_estimate.inverse(), &err, num_points*2);

#ifdef USE_ESM
  // is this the jacobian at true minimum?
  Eigen::Matrix<double, Eigen::Dynamic, 6> M_0(num_points*4, 6);
  Eigen::Matrix<double, 6, 1> zero_vec;
  zero_vec.setZero();
  computeProjectionJacobian(zero_vec, fx, px, py, ref_points, &M_0, 0);
  computeReverseProjectionJacobian(zero_vec, fx, px, py, target_points, &M_0, num_points*2);
#endif

  double initial_sse = err.cwiseProduct(err).sum();
  double final_sse = initial_sse;
  dbg("T0 : [%7.3f %7.3f %7.3f] R: [%7.3f %7.3f %7.3f] E: %f\n", 
      estimate_vec(0), estimate_vec(1), estimate_vec(2), 
      estimate_vec(3), estimate_vec(4), estimate_vec(5), final_sse);

  Eigen::Matrix<double, Eigen::Dynamic, 6> M(num_points * 4, 6);

  for(int iter_num=0; iter_num<max_iterations; iter_num++) {
    // jacobian of target point cloud projected on to reference image
    computeProjectionJacobian(estimate_vec, fx, px, py, target_points, &M, 0);

    // jacobian of reference point cloud projected on to target image
    computeReverseProjectionJacobian(estimate_vec, fx, px, py, ref_points, &M, num_points*2);

#ifdef USE_ESM
    // ESM:
    //    delta = - 2 * pseudoinverse(M + M_0) * error
    M += M_0;
    Eigen::Matrix<double, 6, 1> g = M.transpose() * err;
    Eigen::Matrix<double, 6, 6> MtM = M.transpose() * M;
    Eigen::Matrix<double, 6, 1> delta = - 2 * MtM.inverse() * g;
#else
    // Gauss-Newton:
    //    delta = -(pseudoinverse of M) * error
    //          = - inv(M'*M) * M' * error
    Eigen::Matrix<double, 6, 1> g = M.transpose() * err;
    Eigen::Matrix<double, 6, 6> MtM = M.transpose() * M;
    Eigen::Matrix<double, 6, 1> delta = - MtM.inverse() * g;
#endif

    // compute new isometry estimate
    Eigen::Matrix<double, 6, 1> next_params = estimate_vec + delta;
    Eigen::Isometry3d next_estimate = isometryFromParams(next_params);

    // compute reprojection error at new estimate
    computeReprojectionError(target_points, ref_projections, xyz_c_to_uvw_c, next_estimate, 
        &err, 0);
    computeReprojectionError(ref_points, target_projections, xyz_c_to_uvw_c, next_estimate.inverse(), 
        &err, num_points*2);
    double sse = err.cwiseProduct(err).sum();
    //std::cerr << iter_num << ": " << next_params.transpose() << " : " << sse << std::endl;

    // if stepping would increase the error, then just give up.
    if(sse > final_sse)
      break;

    // update estimate parameters and error values
    estimate_vec = next_params;
    final_sse = sse;
    *result = next_estimate;

    // stop if we're not moving that much
    if(fabs(delta(0)) < 0.0001 &&
       fabs(delta(1)) < 0.0001 &&
       fabs(delta(2)) < 0.0001 &&
       fabs(delta(3)) < (0.01 * M_PI/180) &&
       fabs(delta(4)) < (0.01 * M_PI/180) &&
       fabs(delta(5)) < (0.01 * M_PI/180))
        break;

    dbg("T%-2d: [%7.3f %7.3f %7.3f] R: [%7.3f %7.3f %7.3f] E: %f\n", 
        iter_num+1,
        estimate_vec(0), estimate_vec(1), estimate_vec(2), 
        estimate_vec(3), estimate_vec(4), estimate_vec(5), final_sse);
  }
  dbg("T--: [%7.3f %7.3f %7.3f] R: [%7.3f %7.3f %7.3f] E: %f\n", 
      estimate_vec(0), estimate_vec(1), estimate_vec(2), 
      estimate_vec(3), estimate_vec(4), estimate_vec(5), final_sse);

  // compute the motion estimate covariance.
  //
  // XXX: this assumes that the covariance of the target feature locations is
  // identity.  In the future, we should allow the user to pass in a covariance
  // matrix on the feature locations (or at least specify a covariance for each
  // feature), which would then factor into this covariance matrix computation
  // here.
  if(result_covariance) {
    computeProjectionJacobian(estimate_vec, fx, px, py, target_points, &M, 0);
    computeReverseProjectionJacobian(estimate_vec, fx, px, py, ref_points, &M, num_points*2);
#ifdef USE_ESM
    M += M_0;
    Eigen::Matrix<double, 6, 6> MtM_inv = (M.transpose() * M).inverse();
    *result_covariance = 4 * MtM_inv;
#else
    Eigen::Matrix<double, 6, 6> MtM_inv = (M.transpose() * M).inverse();
    *result_covariance = MtM_inv;
#endif
  }
}

}
