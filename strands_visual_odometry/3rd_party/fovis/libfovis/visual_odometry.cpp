// visual odometry, combining ideas from a number of different visual odometry
// algorithms
//
//   Andrew Howard, "Real-Time Stereo Visual Odometry for Autonomous Ground Vehicles,"
//   International Conference on Robots and Systems (IROS), September 2008
//
//   David Nister
//
//   Sibley et al.
//
//   TODO
//
// Modifications include:
//  - multi-resolution odometry using gaussian pyramids
//
//  - adaptive feature detector thresholding
//
//  - subpixel refinement of feature matches
//
//  TODO

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "tictoc.hpp"
#include "visual_odometry.hpp"
#include "initial_homography_estimation.hpp"
#include "internal_utils.hpp"

#include <iostream>
#include <iomanip>

//#define dbg(...) fprintf(stderr, __VA_ARGS__)
#define dbg(...)
#define dump(var) (std::cerr<<" "#var<<" =[\n"<< std::setprecision (12)<<var<<"]"<<std::endl)

#ifndef MIN
#define MIN(a,b) ((a)<(b) ? (a) : (b))
#endif

namespace fovis
{

static inline int clamp(int val, int minval, int maxval)
{
  if(val > maxval) return maxval;
  if(val < minval) return minval;
  return val;
}

static void validateOptions(const VisualOdometryOptions& options,
    const VisualOdometryOptions& defaults);

// =================== VisualOdometry ===================

VisualOdometry::VisualOdometry(const Rectification* rectification,
                               const VisualOdometryOptions& options) :
    _options(options)
{
  _ref_frame = NULL;
  _prev_frame = NULL;
  _cur_frame = NULL;

  _change_reference_frames = false;

  _ref_frame_change_threshold = 150;

  _frame_count = 0;

  // check for any unrecognized options
  const VisualOdometryOptions& defaults = getDefaultOptions();
  validateOptions(options, defaults);

  // extract options
  _feature_window_size = optionsGetIntOrFromDefault(_options, "feature-window-size", defaults);
  _num_pyramid_levels = optionsGetIntOrFromDefault(_options, "max-pyramid-level", defaults);
  _target_pixels_per_feature = optionsGetIntOrFromDefault(_options, "target-pixels-per-feature", defaults);
  _ref_frame_change_threshold = optionsGetIntOrFromDefault(_options, "ref-frame-change-threshold", defaults);
  _use_homography_initialization = optionsGetBoolOrFromDefault(_options, "use-homography-initialization", defaults);
  _fast_threshold = optionsGetIntOrFromDefault(_options, "fast-threshold", defaults);
  _use_adaptive_threshold = optionsGetBoolOrFromDefault(_options, "use-adaptive-threshold", defaults);
  _fast_threshold_adaptive_gain = optionsGetDoubleOrFromDefault(_options, "fast-threshold-adaptive-gain", defaults);

  _fast_threshold_min = 5;
  _fast_threshold_max = 70;

  _p = new VisualOdometryPriv();

  _p->motion_estimate.setIdentity();
  _p->motion_estimate_covariance.setIdentity(6, 6);
  _p->pose.setIdentity();

  _p->ref_to_prev_frame.setIdentity();

  _rectification = rectification;

  _ref_frame = new OdometryFrame(_rectification, options);

  _prev_frame = new OdometryFrame(_rectification, options);

  _cur_frame = new OdometryFrame(_rectification, options);

  _estimator = new MotionEstimator(_rectification, _options);
}

VisualOdometry::~VisualOdometry()
{
  delete _estimator;
  delete _ref_frame;
  delete _prev_frame;
  delete _cur_frame;
  delete _p;
  _ref_frame = NULL;
  _prev_frame = NULL;
  _cur_frame = NULL;
  tictoc_print_stats(TICTOC_AVG);
}

// Estimate an initial rotation by finding the 2D homography that best aligns
// a template image (the previous image) with the current image.  From this
// homography, extract initial rotation parameters.
Eigen::Quaterniond
VisualOdometry::estimateInitialRotation(const OdometryFrame* prev, const OdometryFrame* cur,
    const Eigen::Isometry3d &init_motion_estimate)
{
  _initial_rotation_pyramid_level = 4;
  int num_pyr_levels = prev->getNumLevels();
  int pyrLevel = MIN(num_pyr_levels-1,_initial_rotation_pyramid_level);
  const PyramidLevel * ref_level = prev->getLevel(pyrLevel);
  const PyramidLevel * target_level = cur->getLevel(pyrLevel);

  InitialHomographyEstimator rotation_estimator;
  rotation_estimator.setTemplateImage(ref_level->getGrayscaleImage(),
      ref_level->getWidth(), ref_level->getHeight(),
      ref_level->getGrayscaleImageStride(),
      _initial_rotation_pyramid_level - pyrLevel);

  rotation_estimator.setTestImage(target_level->getGrayscaleImage(),
      target_level->getWidth(), target_level->getHeight(),
      target_level->getGrayscaleImageStride(),
      _initial_rotation_pyramid_level - pyrLevel);

  Eigen::Matrix3f H = Eigen::Matrix3f::Identity();
  double finalRMS = 0;
  H = rotation_estimator.track(H,8, &finalRMS);
  double scale_factor = 1 << _initial_rotation_pyramid_level;
  Eigen::Matrix3f S = Eigen::Matrix3f::Identity() * scale_factor;
  S(2, 2) = 1;
  //scale H up to the full size image
  H = S * H * S.inverse();
  _p->initial_homography_est = H.cast<double>();
  //    dump(H);

  //TODO: use a better method to get the rotation from homography.
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  const CameraIntrinsicsParameters& input_camera = _rectification->getInputCameraParameters();
  rpy(0) = asin(H(1, 2) / input_camera.fx);
  rpy(1) = -asin(H(0, 2) / input_camera.fx);
  rpy(2) = -atan2(H(1, 0), H(0, 0));

  //    Eigen::Vector3d rpy_deg = rpy.transpose() * 180 / M_PI;
  //    dbg("irot:(% 6.3f % 6.3f % 6.3f)",rpy_deg(0),rpy_deg(1),rpy_deg(2));

  Eigen::Quaterniond q = _rpy_to_quat(rpy);
  return q;
}

void
VisualOdometry::processFrame(const uint8_t* gray, DepthSource* depth_source)
{
  if(_change_reference_frames) {
    // current frame becomes the reference frame
    std::swap(_ref_frame, _cur_frame);
    _p->ref_to_prev_frame.setIdentity();
  } else {
    // reference frame doesn't change, current frame is now previous
    std::swap(_prev_frame, _cur_frame);
  }

  bool changed_reference_frames = _change_reference_frames;

  // initialize a null motion estimate
  _p->motion_estimate.setIdentity();
  _change_reference_frames = false;

  // detect features in new frame
  _cur_frame->prepareFrame(gray, _fast_threshold, depth_source);

  const CameraIntrinsicsParameters& input_camera = _rectification->getInputCameraParameters();
  int width = input_camera.width;
  int height = input_camera.height;

  if (_use_adaptive_threshold) {
    // adaptively adjust feature detector threshold based on how many features
    // were detected.  Use proportional control
    int target_num_features = width * height / _target_pixels_per_feature;
    int err = _cur_frame->getNumDetectedKeypoints() - target_num_features;
    int thresh_adjustment = (int)((err) * _fast_threshold_adaptive_gain);
    _fast_threshold += thresh_adjustment;
    _fast_threshold = clamp(_fast_threshold, _fast_threshold_min, _fast_threshold_max);
  //  dbg("Next FAST threshold: %d (%d)\n", _fast_threshold, thresh_adjustment);
  }

  _frame_count++;

  // Only do the temporal matching if we have feature descriptors from the
  // previous frame.
  if(_frame_count < 2) {
    _change_reference_frames = true;
    return;
  }

  //TODO: should add option to pass in the initial estimate from external source
  tictoc("estimateInitialRotation");
  Eigen::Quaterniond init_rotation_est;
  if (_use_homography_initialization) {
    if (changed_reference_frames) {
      //TODO:this is ugly, but necessary due cuz of swapping stuff above :-/
      init_rotation_est = estimateInitialRotation(_ref_frame, _cur_frame);
    } else {
      init_rotation_est = estimateInitialRotation(_prev_frame, _cur_frame);
    }
  } else {
    init_rotation_est = Eigen::Quaterniond(1, 0, 0, 0); // identity quaternion.
  }

  tictoc("estimateInitialRotation");
  _p->initial_motion_estimate = _p->ref_to_prev_frame.inverse();
  _p->initial_motion_estimate.rotate(init_rotation_est);

  _p->initial_motion_cov.setIdentity();
  //TODO:estimate the covariance

  _estimator->estimateMotion(_ref_frame,
                             _cur_frame,
                             depth_source,
                             _p->initial_motion_estimate,
                             _p->initial_motion_cov);

  if(_estimator->isMotionEstimateValid()) {
    Eigen::Isometry3d to_reference = _estimator->getMotionEstimate();
    _p->motion_estimate = _p->ref_to_prev_frame * to_reference;
    Eigen::MatrixXd to_reference_cov = _estimator->getMotionEstimateCov();
    _p->motion_estimate_covariance = to_reference_cov; //TODO: this should probably be transformed as well
    _p->ref_to_prev_frame = to_reference.inverse();
    _p->pose = _p->pose * _p->motion_estimate;
  } else if(!changed_reference_frames) {
    // if the motion estimation failed, then try estimating against the
    // previous frame if it's not the reference frame.
    dbg("  Failed against reference frame, trying previous frame...\n");
    _p->initial_motion_estimate.setIdentity();
    _p->initial_motion_estimate.rotate(init_rotation_est);
    _p->initial_motion_cov.setIdentity();
    //TODO:covariance?
    _estimator->estimateMotion(_prev_frame,
                               _cur_frame,
                               depth_source,
                               _p->initial_motion_estimate,
                               _p->initial_motion_cov);

    if(_estimator->isMotionEstimateValid()) {
      dbg("   ok, matched against previous frame.\n");
      _p->motion_estimate = _estimator->getMotionEstimate();
      _p->motion_estimate_covariance = _estimator->getMotionEstimateCov();
      _p->pose = _p->pose * _p->motion_estimate;
      _change_reference_frames = true;
    }

  }

  // switch reference frames?
  if(!_estimator->isMotionEstimateValid() ||
      _estimator->getNumInliers() < _ref_frame_change_threshold) {
    _change_reference_frames = true;
  }
  if(_change_reference_frames)
    dbg("Changing reference frames\n");
}

void
VisualOdometry::sanityCheck() const
{
  _cur_frame->sanityCheck();
  _ref_frame->sanityCheck();
  _estimator->sanityCheck();
}

VisualOdometryOptions
VisualOdometry::getDefaultOptions()
{
  VisualOdometryOptions r;
  //TODO split defaults?

  // VisualOdometry, OdometryFrame
  r["feature-window-size"] = "9";
  r["max-pyramid-level"] = "3";
  r["min-pyramid-level"] = "0";
  r["target-pixels-per-feature"] = "250";
  r["fast-threshold"] = "20";
  r["use-adaptive-threshold"] = "true";
  r["fast-threshold-adaptive-gain"] = "0.005";
  r["use-homography-initialization"] = "true";
  r["ref-frame-change-threshold"] = "150";

  // OdometryFrame
  r["use-bucketing"] = "true";
  r["bucket-width"] = "80";
  r["bucket-height"] = "80";
  r["max-keypoints-per-bucket"] = "25";
  r["use-image-normalization"] = "false";

  // MotionEstimator
  r["inlier-max-reprojection-error"] = "1.5";
  r["clique-inlier-threshold"] = "0.1";
  r["min-features-for-estimate"] = "10";
  r["max-mean-reprojection-error"] = "10.0";
  r["use-subpixel-refinement"] = "true";
  r["feature-search-window"] = "25";
  r["update-target-features-with-refined"] = "false";

  // StereoDepth
  r["stereo-require-mutual-match"] = "true";
  r["stereo-max-dist-epipolar-line"] = "1.5";
  r["stereo-max-refinement-displacement"] = "1.0";
  r["stereo-max-disparity"] = "128";

  return r;
}

static void
validateOptions(const VisualOdometryOptions& options,
    const VisualOdometryOptions& defaults)
{
  VisualOdometryOptions::const_iterator oiter = options.begin();
  VisualOdometryOptions::const_iterator oend = options.end();
  for(; oiter != oend; ++oiter) {
    if(defaults.find(oiter->first) == defaults.end()) {
      fprintf(stderr, "VisualOdometry WARNING: unrecognized option [%s]\n", oiter->first.c_str());
    }
  }

  // TODO check option value ranges
}


}
