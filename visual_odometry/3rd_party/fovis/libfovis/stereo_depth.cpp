#include "stereo_depth.hpp"

#include <cstdio>
#include <iostream>

#include <emmintrin.h>

#include "visual_odometry.hpp"
#include "tictoc.hpp"
#include "fast.hpp"
#include "sad.hpp"
#include "rectification.hpp"
#include "feature_match.hpp"
#include "refine_feature_match.hpp"
#include "refine_motion_estimate.hpp"
#include "internal_utils.hpp"
#include "stereo_rectify.hpp"

#define MIN_DISPARITY 0

namespace fovis
{

StereoCalibration::StereoCalibration(const StereoCalibrationParameters& params) :
    _parameters(params)
{
  initialize();
}

StereoCalibration::~StereoCalibration()
{
  delete _left_rectification;
  delete _right_rectification;
}

void
StereoCalibration::initialize()
{
  Eigen::Quaterniond rotation_quat(_parameters.right_to_left_rotation[0],
                                   _parameters.right_to_left_rotation[1],
                                   _parameters.right_to_left_rotation[2],
                                   _parameters.right_to_left_rotation[3]);
  Eigen::Vector3d translation(_parameters.right_to_left_translation[0],
                              _parameters.right_to_left_translation[1],
                              _parameters.right_to_left_translation[2]);

  Eigen::Matrix3d left_rotation, right_rotation;
  stereo_rectify(_parameters.left_parameters, _parameters.right_parameters,
                 rotation_quat,
                 translation,
                 &left_rotation, &right_rotation, &_rectified_parameters);

  _left_rectification = new Rectification(_parameters.left_parameters,
                                          left_rotation,
                                          _rectified_parameters);

  _right_rectification = new Rectification(_parameters.right_parameters,
                                           right_rotation,
                                           _rectified_parameters);
}

StereoCalibration* StereoCalibration::makeCopy() const {
  StereoCalibration* sc = new StereoCalibration();
  sc->_parameters = _parameters;
  sc->_rectified_parameters = _rectified_parameters;
  sc->_left_rectification = _left_rectification->makeCopy();
  sc->_right_rectification = _right_rectification->makeCopy();
  return sc;
}

StereoDepth::StereoDepth(const StereoCalibration* calib,
                         const VisualOdometryOptions& options) :
    _calib(calib),
    _width(calib->getWidth()),
    _height(calib->getHeight()),
    _fast_threshold_min(5),
    _fast_threshold_max(70),
    _options(options)
{

  const VisualOdometryOptions& defaults(VisualOdometry::getDefaultOptions());

  _feature_window_size = optionsGetIntOrFromDefault(_options, "feature-window-size", defaults);
  _num_pyramid_levels = optionsGetIntOrFromDefault(_options, "max-pyramid-level", defaults);
  _fast_threshold = optionsGetIntOrFromDefault(_options, "fast-threshold", defaults);
  _use_adaptive_threshold = optionsGetBoolOrFromDefault(_options, "use-adaptive-threshold", defaults);
  _max_refinement_displacement = optionsGetDoubleOrFromDefault(_options, "stereo-max-refinement-displacement", defaults);
  _require_mutual_match = optionsGetBoolOrFromDefault(_options, "stereo-require-mutual-match", defaults);
  _max_disparity = optionsGetIntOrFromDefault(_options, "stereo-max-disparity", defaults);
  _max_dist_epipolar_line = optionsGetDoubleOrFromDefault(_options, "stereo-max-dist-epipolar-line", defaults);
  _target_pixels_per_feature = optionsGetIntOrFromDefault(_options, "target-pixels-per-feature", defaults);
  _fast_threshold_adaptive_gain = optionsGetDoubleOrFromDefault(_options, "fast-threshold-adaptive-gain", defaults);

  _matched_right_keypoints_per_level.resize(_num_pyramid_levels);

  _right_frame = new StereoFrame(_width, _height, calib->getRightRectification(), _options);

  _uvd1_to_xyz = new Eigen::Matrix4d(calib->getUvdToXyz());

  _matches_capacity = 200;
  _matches = new FeatureMatch[_matches_capacity];
  _num_matches = 0;
}

StereoDepth::~StereoDepth()
{
  delete _right_frame;
  delete[] _matches;
}

void
StereoDepth::setRightImage(const uint8_t * img)
{
  _right_frame->prepareFrame(img, _fast_threshold);
  if (_use_adaptive_threshold) {
    // adaptively adjust feature detector threshold based on how many features
    // were detected.  Use proportional control
    int target_num_features = (_width * _height) / _target_pixels_per_feature;
    int err = _right_frame->getNumDetectedKeypoints() - target_num_features;
    int thresh_adjustment = (int)((err) * _fast_threshold_adaptive_gain);
    _fast_threshold += thresh_adjustment;
    _fast_threshold = (_fast_threshold < _fast_threshold_min)? _fast_threshold_min :
        (_fast_threshold > _fast_threshold_max)? _fast_threshold_max : _fast_threshold;
  }
}

void
StereoDepth::leftRightMatch(PyramidLevel *left_level,
                            PyramidLevel *right_level,
                            Points2d* matched_right_keypoints)
{
  tictoc("leftRightMatch");

  matched_right_keypoints->clear();

  int num_kp_left = left_level->getNumKeypoints();
  int num_kp_right = right_level->getNumKeypoints();
  int level_num = left_level->getLevelNum();

  float adj_max_dist_epipolar_line = _max_dist_epipolar_line*(1 << level_num);

  //assert (left_level->getNumLevel() == right_level->getNumLevel());
  _legal_matches.resize(num_kp_left);
  for (int left_kp_ind = 0; left_kp_ind < num_kp_left; ++left_kp_ind) {
    Eigen::Vector2d ref_rect_base_uv = left_level->getKeypointRectBaseUV(left_kp_ind);
    std::vector<int>& left_candidates(_legal_matches[left_kp_ind]);
    left_candidates.clear();
    for (int right_kp_ind=0; right_kp_ind < num_kp_right; ++right_kp_ind) {
      Eigen::Vector2d diff = ref_rect_base_uv - right_level->getKeypointRectBaseUV(right_kp_ind);
      // TODO some sort of binary search
      if (diff(1) < -adj_max_dist_epipolar_line) { break; }
      // epipolar and disparity constraints
      if ((fabs(diff(1)) < adj_max_dist_epipolar_line) &&
          (diff(0) > MIN_DISPARITY) &&
          (diff(0) < _max_disparity)) {
        left_candidates.push_back(right_kp_ind);
      }
    }
  }

  int max_num_matches = std::min(num_kp_left, num_kp_right);
  if (_matches_capacity < max_num_matches) {
    _matches_capacity = static_cast<int>(1.2*max_num_matches);
    delete[] _matches;
    _matches = new FeatureMatch[_matches_capacity];
  }
  _num_matches = 0;
  _matcher.matchFeatures(left_level, right_level, _legal_matches, &_matches[0], &_num_matches);

  // subpixel refinement on correspondences
  for (int n=0; n < _num_matches; ++n) {
    FeatureMatch& match(_matches[n]);

    KeypointData* left_kpdata(match.ref_keypoint);
    KeypointData* right_kpdata(match.target_keypoint);

    Eigen::Vector2d left_uv(left_kpdata->kp.u, left_kpdata->kp.v);
    Eigen::Vector2d init_right_uv(right_kpdata->kp.u, right_kpdata->kp.v);

    Eigen::Vector2d refined_right_uv;
    float delta_sse = -1;
    refineFeatureMatch(left_level, right_level, left_uv, init_right_uv,
                       &refined_right_uv, &delta_sse);
    double ds = (init_right_uv - refined_right_uv).norm();
    Eigen::Vector2d refined_right_base_uv = refined_right_uv * (1 << level_num);

    Eigen::Vector2d rect_refined_right_base_uv;
    _right_frame->rectify(refined_right_base_uv, &rect_refined_right_base_uv);

    Eigen::Vector2d diff = left_kpdata->rect_base_uv - rect_refined_right_base_uv;
    double disparity = diff(0);

    // re-enforce disparity constraints, epipolar constraints, and excessive
    // refinement displacement
    if ((disparity < MIN_DISPARITY) ||
        (disparity > _max_disparity) ||
        (ds > _max_refinement_displacement) ||
        (fabs(diff(1)) > adj_max_dist_epipolar_line)) {
      left_kpdata->has_depth = false;
      left_kpdata->xyzw = Eigen::Vector4d(NAN, NAN, NAN, NAN);
      left_kpdata->xyz = Eigen::Vector3d(NAN, NAN, NAN);
      left_kpdata->disparity = NAN;
      continue;
    }

    Eigen::Vector4d uvd1(left_kpdata->rect_base_uv(0),
                         left_kpdata->rect_base_uv(1),
                         disparity,
                         1);

    left_kpdata->xyzw = (*_uvd1_to_xyz) * uvd1;
    left_kpdata->xyz = left_kpdata->xyzw.head<3>() / left_kpdata->xyzw.w();
    left_kpdata->has_depth = true;
    left_kpdata->disparity = disparity;

    // TODO we are relying on matches being ordered by increasing left keypoint index.
    matched_right_keypoints->push_back(std::make_pair(refined_right_uv(0), refined_right_uv(1)));
  }

  tictoc("leftRightMatch");
}

bool
StereoDepth::haveXyz(int u, int v)
{
  return true;
}

void
StereoDepth::getXyz(OdometryFrame * odom_frame)
{
  int num_levels = odom_frame->getNumLevels();
  for(int level_num=0; level_num < num_levels; ++level_num) {
    PyramidLevel* left = odom_frame->getLevel(level_num);
    PyramidLevel* right = _right_frame->getLevel(level_num);
    Points2d* matched_right_keypoints(&_matched_right_keypoints_per_level.at(level_num));
    leftRightMatch(left, right, matched_right_keypoints);
  }
}

void
StereoDepth::refineXyz(FeatureMatch * matches,
                       int num_matches,
                       OdometryFrame * odom_frame)
{
  for (int m_ind = 0; m_ind < num_matches; ++m_ind) {
    FeatureMatch& match = matches[m_ind];
    if (match.status != MATCH_NEEDS_DEPTH_REFINEMENT) {
      continue;
    }

    const KeypointData * left_kpdata = match.target_keypoint;
    int level_num = match.ref_keypoint->pyramid_level;
    int kp_ind = left_kpdata->keypoint_index;

    // TODO this is brittle. We are relying too much on what
    // external code does with the keypoints in the odom_frame
    // (ie that keypoints without depth are deleted).
    Eigen::Vector2d left_uv(match.refined_target_keypoint.kp.u,
                            match.refined_target_keypoint.kp.v);

    Eigen::Vector2d right_uv(_matched_right_keypoints_per_level[level_num][kp_ind].first,
                             _matched_right_keypoints_per_level[level_num][kp_ind].second);

    PyramidLevel* left_level = odom_frame->getLevel(level_num);
    PyramidLevel* right_level = _right_frame->getLevel(level_num);

    Eigen::Vector2d refined_right_uv;
    float delta_sse;
    refineFeatureMatch(left_level, right_level, left_uv, right_uv,
                       &refined_right_uv, &delta_sse);

    double ds = (right_uv - refined_right_uv).norm();

    // scale refined and unrefined up to base level dimensions
    Eigen::Vector2d base_refined_right_uv = refined_right_uv * (1 << level_num);

    // get rectified and undistorted position
    Eigen::Vector2d rect_base_refined_right_uv;
    _right_frame->rectify(base_refined_right_uv, &rect_base_refined_right_uv);

    Eigen::Vector2d diff = match.refined_target_keypoint.rect_base_uv - rect_base_refined_right_uv;
    double disparity = diff(0);
    if (disparity < MIN_DISPARITY || disparity > _max_disparity ||
        ds > _max_refinement_displacement ||
        fabs(diff(1)) > _max_dist_epipolar_line*(1 << level_num)) {
      match.status = MATCH_REFINEMENT_FAILED;
      match.inlier = false;
      match.refined_target_keypoint.xyzw = Eigen::Vector4d(NAN, NAN, NAN, NAN);
      match.refined_target_keypoint.xyz = Eigen::Vector3d(NAN, NAN, NAN);
      match.refined_target_keypoint.disparity = NAN;
      continue;
    }

    KeypointData& kpdata(match.refined_target_keypoint);
    Eigen::Vector4d uvd1(kpdata.rect_base_uv(0), kpdata.rect_base_uv(1), disparity, 1);
    kpdata.xyzw = (*_uvd1_to_xyz) * uvd1;
    kpdata.xyz = kpdata.xyzw.head<3>() / kpdata.xyzw.w();
    kpdata.disparity = disparity;
    match.status = MATCH_OK;
  }

}

} /*  */
