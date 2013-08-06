#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include <iostream>
#include <iomanip>

#include "motion_estimation.hpp"
#include "absolute_orientation_horn.hpp"
#include "depth_source.hpp"
#include "refine_motion_estimate.hpp"
#include "tictoc.hpp"
#include "internal_utils.hpp"
#include "sad.hpp"
#include "visual_odometry.hpp"
#include "refine_feature_match.hpp"

#include "stereo_depth.hpp"

#define USE_HORN_ABSOLUTE_ORIENTATION
#define USE_ROBUST_STEREO_COMPATIBILITY
#define USE_BIDIRECTIONAL_REFINEMENT

#define dbg(...) fprintf(stderr, __VA_ARGS__)
#define dump(var) (std::cerr<<" "#var<<" =[\n"<< std::setprecision (12)<<var<<"]"<<std::endl)

namespace fovis {

const char* MotionEstimateStatusCodeStrings[] = {
  "NO_DATA",
  "SUCCESS",
  "INSUFFICIENT_INLIERS",
  "OPTIMIZATION_FAILURE",
  "REPROJECTION_ERROR"
};

MotionEstimator::MotionEstimator(const Rectification* rectification,
    const VisualOdometryOptions& options)
{
  _rectification = rectification;

  _ref_frame = NULL;
  _target_frame = NULL;

  _num_inliers = 0;
  _motion_estimate = new Eigen::Isometry3d();
  _motion_estimate_covariance = new Eigen::MatrixXd();
  _motion_estimate->setIdentity();
  _estimate_status = NO_DATA;
  _motion_estimate_covariance->setIdentity(6,6);

  _matches = NULL;
  _num_matches = 0;
  _matches_capacity = 0;
  _num_tracks = 0;
  _num_frames = 0;

  // extract options
  VisualOdometryOptions defaults = VisualOdometry::getDefaultOptions();
  _inlier_max_reprojection_error = optionsGetDoubleOrFromDefault(options, "inlier-max-reprojection-error", defaults);
  _clique_inlier_threshold = optionsGetDoubleOrFromDefault(options, "clique-inlier-threshold", defaults);
  _min_features_for_valid_motion_estimate = optionsGetIntOrFromDefault(options, "min-features-for-estimate", defaults);
  _max_mean_reprojection_error = optionsGetDoubleOrFromDefault(options, "max-mean-reprojection-error", defaults);
  _use_subpixel_refinement = optionsGetBoolOrFromDefault(options, "use-subpixel-refinement", defaults);
  _max_feature_motion = optionsGetDoubleOrFromDefault(options, "feature-search-window", defaults);
  _update_target_features_with_refined = _use_subpixel_refinement && optionsGetBoolOrFromDefault(options, "update-target-features-with-refined", defaults);

}

MotionEstimator::~MotionEstimator()
{
  _ref_frame = NULL;
  _target_frame = NULL;
  delete[] _matches;
  _num_matches = 0;
  _matches_capacity = 0;
  _num_inliers = 0;
  delete _motion_estimate_covariance;
  delete _motion_estimate;
  _motion_estimate_covariance = NULL;
  _motion_estimate = NULL;
  _estimate_status = NO_DATA;
}

void
MotionEstimator::estimateMotion(OdometryFrame* ref_frame,
                                OdometryFrame* target_frame,
                                DepthSource* depth_source,
                                const Eigen::Isometry3d &init_motion_est,
                                const Eigen::MatrixXd &init_motion_cov)
{
  tictoc("estimateMotion");

  _depth_source = depth_source;

  _ref_frame = ref_frame;
  _target_frame = target_frame;
  *_motion_estimate = init_motion_est;
  *_motion_estimate_covariance = init_motion_cov;

  //  dbg("Process frame\n");

  tictoc("matchFeatures_all_levels");
  _num_matches = 0;
  int max_num_matches = std::min(_ref_frame->getNumKeypoints(), _target_frame->getNumKeypoints());
  if (max_num_matches > _matches_capacity) {
    delete[] _matches;
    _matches_capacity = static_cast<int>(max_num_matches * 1.2);
    _matches = new FeatureMatch[_matches_capacity];
  }

  int num_levels = _ref_frame->getNumLevels();
  for (int level_ind = 0; level_ind < num_levels; level_ind++) {
    PyramidLevel* ref_level = _ref_frame->getLevel(level_ind);
    PyramidLevel* target_level = _target_frame->getLevel(level_ind);
    matchFeatures(ref_level, target_level);
  }
  if (_use_subpixel_refinement) {
    depth_source->refineXyz(_matches, _num_matches, target_frame);
  }
  tictoc("matchFeatures_all_levels");

  // assign a 'local' match id for use in inlier detection
  for (int i=0; i < _num_matches; ++i) {
    _matches[i].id = i;
  }

  tictoc("computeMaximallyConsistentClique");
  computeMaximallyConsistentClique();
  tictoc("computeMaximallyConsistentClique");

  // first pass at motion estimation
  tictoc("estimateRigidBodyTransform1");
#ifdef USE_ROBUST_STEREO_COMPATIBILITY
  // Horn/SVD doesn't do too well when using robust stereo metric
  _estimate_status = SUCCESS;
#else
  estimateRigidBodyTransform();
#endif
  tictoc("estimateRigidBodyTransform1");

  // refine motion estimate by minimizing reprojection error
  tictoc("refineMotionEstimate");
  refineMotionEstimate();
  tictoc("refineMotionEstimate");

  // compute inlier reprojection error
  tictoc("computeReprojectionError");
  computeReprojectionError();
  tictoc("computeReprojectionError");

  // remove features with a high reprojection error from the inlier set
  _num_reprojection_failures = 0;
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    if (!match.inlier)
      continue;
    if (match.reprojection_error > _inlier_max_reprojection_error) {
      match.inlier = false;
      _num_inliers--;
      _num_reprojection_failures++;
    }
  }

  // prevent propagation of track id's through outlier matches.
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    if (!match.inlier) {
      match.target_keypoint->track_id = -1;
    }
  }

  // second motion estimate refinement
  tictoc("refineMotionEstimate");
  refineMotionEstimate();
  tictoc("refineMotionEstimate");

  // compute new reprojection error
  tictoc("computeReprojectionError");
  computeReprojectionError();
  tictoc("computeReprojectionError");

  if (_mean_reprojection_error > _max_mean_reprojection_error) {
    _estimate_status = REPROJECTION_ERROR;
  }

#if 0
  switch (_estimate_status) {
  case SUCCESS:
    dbg("Inliers: %4d  Rep. fail: %4d Matches: %4d Feats: %4d Mean err: %5.2f ",
        _num_inliers,
        _num_reprojection_failures,
        _num_matches,
        (int) _target_frame->getNumKeypoints(),
        _mean_reprojection_error);
    //    print_isometry(_motion_estimate);
    dbg("\n");
    break;
  case REPROJECTION_ERROR:
    dbg("Excessive reprojection error (%f).\n", _mean_reprojection_error);
    break;
  case OPTIMIZATION_FAILURE:
    dbg("Unable to solve for rigid body transform\n");
    break;
  case INSUFFICIENT_INLIERS:
    dbg("Insufficient inliers\n");
    break;
  default:
    dbg("Unknown error (this should never happen)\n");
    break;
  }
#endif

  if (_update_target_features_with_refined) {
    for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
      FeatureMatch& match = _matches[m_ind];
      match.target_keypoint->copyFrom(match.refined_target_keypoint);
    }
  }

  _num_frames++;

  tictoc("estimateMotion");
}

void MotionEstimator::sanityCheck() const
{
#ifndef NDEBUG
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    const FeatureMatch& match = _matches[m_ind];

    const KeypointData* ref_kp = match.ref_keypoint;
    const KeypointData* target_kp = match.target_keypoint;
    assert(ref_kp->pyramid_level >= 0 &&
           ref_kp->pyramid_level < _ref_frame->getNumLevels());
    assert(target_kp->pyramid_level >= 0 &&
           target_kp->pyramid_level < _target_frame->getNumLevels());
    const PyramidLevel * ref_level = _ref_frame->getLevel(ref_kp->pyramid_level);
    const PyramidLevel * target_level = _target_frame->getLevel(target_kp->pyramid_level);
    assert(ref_kp->kp.u >= 0 && ref_kp->kp.v >= 0 &&
           ref_kp->kp.u < ref_level->getWidth() &&
           ref_kp->kp.v < ref_level->getHeight());
    assert(target_kp->kp.u >= 0 && target_kp->kp.v >= 0 &&
           target_kp->kp.u < target_level->getWidth() &&
           target_kp->kp.v < target_level->getHeight());
  }
#endif
}

void MotionEstimator::matchFeatures(PyramidLevel* ref_level, PyramidLevel* target_level)
{
  // get the camera projection matrix
  Eigen::Matrix<double, 3, 4> xyz_c_to_uvw_c =
    _rectification->getRectifiedCameraParameters().toProjectionMatrix();
  // get the ref_to_target isometry cuz of order of loops
  Eigen::Isometry3d ref_to_target = _motion_estimate->inverse();
  Eigen::Matrix<double, 3, 4> reproj_mat = xyz_c_to_uvw_c * ref_to_target.matrix();
  int num_ref_features = ref_level->getNumKeypoints();
  int num_target_features = target_level->getNumKeypoints();

  std::vector<std::vector<int> > candidates(num_ref_features);
  for (int ref_ind = 0; ref_ind < num_ref_features; ref_ind++) {
    // constrain the matching to a search-region based on the
    // current motion estimate
    const Eigen::Vector4d& ref_xyzw = ref_level->getKeypointXYZW(ref_ind);
    assert(!isnan(ref_xyzw(0)) && !isnan(ref_xyzw(1)) &&
           !isnan(ref_xyzw(2)) && !isnan(ref_xyzw(3)));
    Eigen::Vector3d reproj_uv1 = reproj_mat * ref_xyzw;
    reproj_uv1 /= reproj_uv1(2);
    Eigen::Vector2d ref_uv(ref_level->getKeypointRectBaseU(ref_ind),
                           ref_level->getKeypointRectBaseV(ref_ind));
    std::vector<int>& ref_candidates(candidates[ref_ind]);
    for (int target_ind = 0; target_ind < num_target_features; target_ind++) {
      Eigen::Vector2d target_uv(target_level->getKeypointRectBaseU(target_ind),
                                target_level->getKeypointRectBaseV(target_ind));
      //TODO: Should adapt based on covariance instead of constant sized window!
      //Eigen::Vector2d err = target_uv - ref_uv; //ignore motion est
      Eigen::Vector2d err = target_uv - reproj_uv1.head<2>();
      if (err.norm() < _max_feature_motion) {
        ref_candidates.push_back(target_ind);
      }
    }
  }

  int inserted_matches = 0;
  _matcher.matchFeatures(ref_level, target_level, candidates,
                         &(_matches[_num_matches]), &inserted_matches);
  int old_num_matches = _num_matches;
  _num_matches = old_num_matches + inserted_matches;

  if (_use_subpixel_refinement) {
    for (int n=old_num_matches; n < _num_matches; ++n) {
      //std::cerr << "n = " << n << std::endl;
      FeatureMatch& match(_matches[n]);
      const KeypointData* ref_kpdata(match.ref_keypoint);
      const KeypointData* target_kpdata(match.target_keypoint);
      Eigen::Vector2d ref_uv(ref_kpdata->kp.u, ref_kpdata->kp.v);
      Eigen::Vector2d init_target_uv(target_kpdata->kp.u, target_kpdata->kp.v);
      Eigen::Vector2d final_target_uv;

      float delta_sse = -1;
      refineFeatureMatch(ref_level, target_level, ref_uv, init_target_uv,
                         &final_target_uv, &delta_sse);
      double ds = (init_target_uv - final_target_uv).norm();
      if (ds < 1e-9) {
        match.refined_target_keypoint.copyFrom(*match.target_keypoint);
        match.status = MATCH_OK;
      } else if (ds > 1.5) {
        // TODO make threshold a parameter. Also, reject or keep?
        match.refined_target_keypoint.copyFrom(*match.target_keypoint);
        match.status = MATCH_OK;
      } else {
        match.refined_target_keypoint.kp.u = final_target_uv(0);
        match.refined_target_keypoint.kp.v = final_target_uv(1);
        match.refined_target_keypoint.base_uv =
          final_target_uv * (1 << target_kpdata->pyramid_level);
        _rectification->rectifyBilinearLookup(match.refined_target_keypoint.base_uv,
            &match.refined_target_keypoint.rect_base_uv);
        match.status = MATCH_NEEDS_DEPTH_REFINEMENT;
      }
    }
  }

  // label matches with their track_id
  for (int n=old_num_matches; n < _num_matches; ++n) {
    FeatureMatch& match(_matches[n]);
    KeypointData* ref_kpdata(match.ref_keypoint);
    KeypointData* target_kpdata(match.target_keypoint);
    if (ref_kpdata->track_id < 0) {
      //ref wasn't already part of a track
      ref_kpdata->track_id = _num_tracks++;
    }
    target_kpdata->track_id = ref_kpdata->track_id;
    match.track_id = ref_kpdata->track_id;
  }

}

// used for sorting feature matches.
static bool consistencyCompare(const FeatureMatch &ca, const FeatureMatch& cb)
{
  return ca.compatibility_degree > cb.compatibility_degree;
}

#ifdef USE_ROBUST_STEREO_COMPATIBILITY
// Robust Stereo Compatibility Check from:
// Heiko Hirschmuller, Peter R. Innocent and Jon M. Garibaldi
// "Fast, Unconstrained Camera Motion Estimation from Stereo without Tracking
// and Robust Statistics"
// Paper recommends a De (compatibility thresh) of ~0.2 pixels

static inline double sqr(double x) { return x * x; }

static inline
double robustStereoCompatibility_computeDL(double L,
                                           const  Eigen::Vector3d & p1,
                                           const  Eigen::Vector3d & p2,
                                           double t, double f, double De)
{
  double A = sqr((p1(0) - p2(0)) * (t - p1(0)) - (p1(1) - p2(1)) * p1(1) - (p1(2) - p2(2)) * p1(2));
  double B = sqr((p1(0) - p2(0)) * p1(0) + (p1(1) - p2(1)) * p1(1) + (p1(2) - p2(2)) * p1(2));
  double C = 0.5 * sqr(t * (p1(1) - p2(1)));
  double D = sqr((p1(0) - p2(0)) * (t - p2(0)) - (p1(1) - p2(1)) * p2(1) - (p1(2) - p2(2)) * p2(2));
  double E = sqr((p1(0) - p2(0)) * p2(0) + (p1(1) - p2(1)) * p2(1) + (p1(2) - p2(2)) * p2(2));
  double F = 0.5 * sqr(t * (p1(1) - p2(1)));
  return De / (L * f * t) * sqrt(sqr(p1(2)) * (A + B + C) + sqr(p2(2)) * (D + E + F));
}

static inline bool
robustStereoCompatibility(const Eigen::Vector3d & C1,
                          const Eigen::Vector3d & C2,
                          const Eigen::Vector3d & P1,
                          const Eigen::Vector3d & P2,
                          double baseline,
                          double focal_length,
                          double De)
{
  //compute the L quantities (ie dist between pairs of points)
  double L1 = (C2-C1).norm();
  double L2 = (P2-P1).norm();
  //compute the DLs (delta L)
  double DL1 = robustStereoCompatibility_computeDL(L1, C1, C2, baseline, focal_length, De);
  double DL2 = robustStereoCompatibility_computeDL(L2, P1, P2, baseline, focal_length, De);
  return (fabs(L1-L2) <= 3*sqrt(sqr(DL1)+sqr(DL2)));
}
#endif

void MotionEstimator::computeMaximallyConsistentClique()
{
  if (!_num_matches)
    return;

  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    match.consistency_vec.resize(_num_matches);
  }

#ifdef USE_ROBUST_STEREO_COMPATIBILITY
  double baseline = _depth_source->getBaseline();
  bool have_baseline = baseline > 0;
  // XXX this is not actually correct for kinect/primesense
  const CameraIntrinsicsParameters& rparams = _rectification->getRectifiedCameraParameters();
  double stereo_focal_length = rparams.fx;
#endif

  // For each pair of matches, compute the distance between features in the
  // reference frame, and the distance between features in the target frame.
  // Rigid body transformations (isometries) preserve distance, so the distance
  // should not change significantly if the two feature matches are
  // "compatible".
  //
  // If the depth comes from a stereo camera, then apply a consistency metric that
  // allows for disparity error resulting from the stereo baseline.

  // FIXME using both homogeneous and cartesian coordinates is a bit gross here...
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];

    const Eigen::Vector4d& ref_xyzw_1 = match.ref_keypoint->xyzw;
    const Eigen::Vector4d& target_xyzw_1 = match.refined_target_keypoint.xyzw;
    const Eigen::Vector3d& ref_xyz_1 = match.ref_keypoint->xyz;
    const Eigen::Vector3d& target_xyz_1 = match.refined_target_keypoint.xyz;
    assert(match.id == m_ind);

    // are the features points at infinity?
    bool ref_infinity = ref_xyzw_1.w() < 1e-9;
    bool target_infinity = target_xyzw_1.w() < 1e-9;

    for (int m_ind2 = m_ind + 1; m_ind2 < _num_matches; m_ind2++) {
      FeatureMatch& match2 = _matches[m_ind2];
      assert(match2.id == m_ind2);
      const Eigen::Vector4d& ref_xyzw_2 = match2.ref_keypoint->xyzw;
      const Eigen::Vector4d& target_xyzw_2 = match2.refined_target_keypoint.xyzw;
      const Eigen::Vector3d& ref_xyz_2 = match2.ref_keypoint->xyz;
      const Eigen::Vector3d& target_xyz_2 = match2.refined_target_keypoint.xyz;

      bool consistent;
      // special case:  if either of the features are points at infinity, then
      // we can't compare their distances.
      if((ref_infinity && ref_xyzw_2.w() < 1e-9) ||
         (target_infinity && target_xyzw_2.w() < 1e-9)) {
        consistent = true;
      } else {
#ifdef USE_ROBUST_STEREO_COMPATIBILITY
        if (have_baseline) {
          consistent = robustStereoCompatibility(ref_xyz_1, ref_xyz_2,
                                                 target_xyz_1 ,target_xyz_2,
                                                 baseline, stereo_focal_length,
                                                 _clique_inlier_threshold);
        } else {
          double ref_dist = (ref_xyz_2 - ref_xyz_1).norm();
          double target_dist = (target_xyz_2 - target_xyz_1).norm();
          consistent = fabs(ref_dist - target_dist) < _clique_inlier_threshold;
        }
#else
        double ref_dist = (ref_xyz_2 - ref_xyz_1).norm();
        double target_dist = (target_xyz_2 - target_xyz_1).norm();
        consistent = fabs(ref_dist - target_dist) < _clique_inlier_threshold;
#endif
      }

      if (consistent) {
        match.consistency_vec[match2.id] = 1;
        match.compatibility_degree++;
        match2.consistency_vec[match.id] = 1;
        match2.compatibility_degree++;
      }
    }
  }

  // sort the features based on their consistency with other features
  std::sort(_matches, _matches+_num_matches, consistencyCompare);

  // pick the best feature and mark it as an inlier
  FeatureMatch &best_candidate = _matches[0];
  best_candidate.in_maximal_clique = true;
  best_candidate.inlier = true;
  _num_inliers = 1;

  // start a list of quick-reject features (features that are known to be
  // inconsistent with any of the existing inliers)
  int reject[_num_matches];
  std::fill(reject, reject+_num_matches, 0);
  for (int m_ind = 1; m_ind < _num_matches; m_ind++) {
    int other_id = _matches[m_ind].id;
    if (!best_candidate.consistency_vec[other_id])
      reject[other_id] = 1;
  }

  // now start adding inliers that are consistent with all existing
  // inliers
  for (int m_ind = 1; m_ind < _num_matches; m_ind++) {
    FeatureMatch& cand = _matches[m_ind];

    // if this candidate is consistent with fewer than the existing number
    // of inliers, then immediately stop iterating since no more features can
    // be inliers
    if (cand.compatibility_degree < _num_inliers)
      break;

    // skip if it's a quick reject
    if (reject[cand.id])
      continue;

    cand.in_maximal_clique = true;
    cand.inlier = true;
    _num_inliers++;

    // mark some more features for rejection
    for (int r_ind = m_ind + 1; r_ind < _num_matches; r_ind++) {
      int other_id = _matches[r_ind].id;
      if (!reject[other_id] && !cand.consistency_vec[other_id])
        reject[other_id] = 1;
    }
  }
}

void MotionEstimator::estimateRigidBodyTransform()
{
  _motion_estimate->setIdentity();

  if (_num_inliers < _min_features_for_valid_motion_estimate) {
    _estimate_status = INSUFFICIENT_INLIERS;
    return;
  }

  // gather all the inliers into two big matrices
  Eigen::MatrixXd target_xyz(3, _num_inliers);
  Eigen::MatrixXd ref_xyz(3, _num_inliers);

  int i = 0;
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    if (!match.inlier)
      continue;
    // FIXME so that this works with points at infinity
    target_xyz.col(i) = match.refined_target_keypoint.xyzw.head<3>() /
        match.refined_target_keypoint.xyzw(3) ;
    ref_xyz.col(i) = match.ref_keypoint->xyzw.head<3>() /
       match.ref_keypoint->xyzw(3);
    i++;
  }

#ifdef USE_HORN_ABSOLUTE_ORIENTATION
  if (0 != absolute_orientation_horn(target_xyz, ref_xyz, _motion_estimate)) {
    _estimate_status = OPTIMIZATION_FAILURE;
    return;
  }
#else
  Eigen::Matrix4d ume_estimate = Eigen::umeyama(target_xyz, ref_xyz);
  *_motion_estimate = Eigen::Isometry3d(ume_estimate);
#endif

  _estimate_status = SUCCESS;
}

void MotionEstimator::refineMotionEstimate()
{

  if (_num_inliers < _min_features_for_valid_motion_estimate) {
    _estimate_status = INSUFFICIENT_INLIERS;
    return;
  }

#ifdef USE_BIDIRECTIONAL_REFINEMENT
  // gather all the inliers into matrices
  Eigen::MatrixXd target_xyz(4,_num_inliers);
  Eigen::MatrixXd target_projections(2,_num_inliers);
  Eigen::MatrixXd ref_xyz(4,_num_inliers);
  Eigen::MatrixXd ref_projections(2,_num_inliers);
  int i = 0;
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    if (!match.inlier)
      continue;
    target_xyz.col(i) = match.refined_target_keypoint.xyzw;
    target_projections.col(i) = match.refined_target_keypoint.rect_base_uv;

    ref_xyz.col(i) = match.ref_keypoint->xyzw;
    ref_projections.col(i) = match.ref_keypoint->rect_base_uv;
    i++;
  }

  const CameraIntrinsicsParameters& rparams = _rectification->getRectifiedCameraParameters();

  // refine motion estimate by minimizing bidirectional reprojection error.
  // bidirectional reprojection error is the error of the target features
  // projected into the reference image, along with the reference features
  // projected into the target image
  refineMotionEstimateBidirectional(ref_xyz,
          ref_projections,
          target_xyz,
          target_projections,
          rparams.fx,
          rparams.cx,
          rparams.cy,
          *_motion_estimate,
          6,
          _motion_estimate,
          _motion_estimate_covariance);

  // TODO regularize the motion estimate covariance.
#else
  // gather all the inliers into matrices
  Eigen::MatrixXd target_xyz(4,_num_inliers);
  Eigen::MatrixXd ref_projections(2,_num_inliers);
  int i = 0;
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    if (!match.inlier)
      continue;
    target_xyz.col(i) = match.refined_target_keypoint.xyzw;

    ref_projections.col(i) = match.ref_keypoint->rect_base_uv;
    i++;
  }

  const CameraIntrinsicsParameters& rparams = _rectification->getRectifiedCameraParameters();
  // refine motion estimate by minimizing reprojection error of the
  // target features projected into the reference image.
  *_motion_estimate = refineMotionEstimate(target_xyz,
          ref_projections,
          rparams->fx,
          rparams->cx,
          rparams->cy,
          *_motion_estimate,
          6);
#endif
}

void MotionEstimator::computeReprojectionError()
{
  if (_estimate_status != SUCCESS)
    return;

  Eigen::Matrix<double, 3, 4> proj_matrix =
    _rectification->getRectifiedCameraParameters().toProjectionMatrix();
  Eigen::Matrix<double, 3, 4> reproj_matrix =
      proj_matrix * _motion_estimate->matrix();

  _mean_reprojection_error = 0;

  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    if (!match.inlier) {
      continue;
    }

    Eigen::Vector3d reproj_homogeneous =
        reproj_matrix * match.refined_target_keypoint.xyzw;

    Eigen::Vector2d reproj(reproj_homogeneous(0) / reproj_homogeneous(2),
        reproj_homogeneous(1) / reproj_homogeneous(2));

    Eigen::Vector2d err = match.ref_keypoint->rect_base_uv - reproj;

    match.reprojection_error = err.norm();

    //    printf("%3d:  %6.1f, %6.1f, %6.1f -> %6.1f, %6.1f -> %6.2f\n", m_ind,
    //        transformed_xyzw(0), transformed_xyzw(1), transformed_xyzw(2),
    //        reprojected_x, reprojected_y,
    //        match.reprojection_error
    //        );
    _mean_reprojection_error += match.reprojection_error;
  }
  _mean_reprojection_error /= _num_inliers;
}

}
