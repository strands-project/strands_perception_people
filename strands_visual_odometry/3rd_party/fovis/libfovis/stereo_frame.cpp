#include "stereo_frame.hpp"

#include <stdio.h>

#include <iostream>

#include "visual_odometry.hpp"
#include "internal_utils.hpp"
#include "gauss_pyramid.h"
#include "fast.hpp"
#include "tictoc.hpp"
#include "rectification.hpp"
#include "normalize_image.hpp"

#ifndef ALIGNMENT
#define ALIGNMENT 16
#endif

namespace fovis
{

static bool
keypoint_rect_v_comparator(const KeypointData& a,
                           const KeypointData& b) {
    return (a.rect_base_uv(1) < b.rect_base_uv(1));
}

StereoFrame::StereoFrame(int width, int height,
                const Rectification* rectify_map,
                const VisualOdometryOptions& options) :
    _base_width(width), _base_height(height)
{

  const VisualOdometryOptions& defaults(VisualOdometry::getDefaultOptions());

  _num_levels = optionsGetIntOrFromDefault(options, "max-pyramid-level", defaults);
  _feature_window_size = optionsGetIntOrFromDefault(options, "feature-window-size", defaults);
  _use_bucketing = optionsGetBoolOrFromDefault(options, "use-bucketing", defaults);
  _use_image_normalization = optionsGetBoolOrFromDefault(options, "use-image-normalization", defaults);

  int bucket_width = optionsGetIntOrFromDefault(options, "bucket-width", defaults);
  int bucket_height = optionsGetIntOrFromDefault(options, "bucket-height", defaults);
  int max_keypoints_per_bucket = optionsGetIntOrFromDefault(options, "max-keypoints-per-bucket", defaults);

  _rectify_map = rectify_map->makeCopy();
  initialize(bucket_width, bucket_height, max_keypoints_per_bucket);
}

StereoFrame::~StereoFrame() {
  for (std::vector<PyramidLevel *>::iterator itr = _levels.begin();
       itr != _levels.end();
       ++itr) {
    delete *itr;
  }
  delete _rectify_map;
}

void
StereoFrame::initialize(int bucket_width,
                        int bucket_height,
                        int max_keypoints_per_bucket) {
  for(int level_num=0; level_num < _num_levels; ++level_num) {

    int level_width = _base_width >> level_num;
    int level_height = _base_height >> level_num;

    GridKeyPointFilter grid_filter(level_width, level_height, bucket_width,
                                   bucket_height, max_keypoints_per_bucket);

    PyramidLevel* level = new PyramidLevel(level_width, level_height,
                                           level_num, _feature_window_size,
                                           grid_filter);
    _levels.push_back(level);
  }
}

void
StereoFrame::prepareFrame(const uint8_t* raw_gray, int fast_threshold) {

  // copy raw image to first pyramid level
  assert(_num_levels);
  PyramidLevel* first_level = _levels[0];
  int src_stride = _base_width;
  for(int row=0; row<_base_height; row++) {
    memcpy(first_level->_raw_gray + row * first_level->_raw_gray_stride,
        raw_gray + row * src_stride, _base_width);
  }

  if (_use_image_normalization) {
    normalize_image(first_level->_raw_gray, first_level->_raw_gray_stride,
                    _base_width, _base_height);
  }

  // compute image pyramid and detect initial features
  for(int level_num=0; level_num<_num_levels; ++level_num) {
    PyramidLevel* level = _levels[level_num];
    if(level_num > 0) {
      // resize the image from the previous level
      PyramidLevel* prev_level = _levels[level_num-1];
      int prev_width = prev_level->getWidth();
      int prev_height = prev_level->getHeight();
      gauss_pyr_down_8u_C1R(prev_level->_raw_gray, prev_level->_raw_gray_stride,
          prev_width, prev_height, level->_raw_gray, level->_raw_gray_stride,
          prev_level->_pyrbuf);
    }

    level->_initial_keypoints.clear();
    FAST(level->_raw_gray, level->_width, level->_height, level->_raw_gray_stride,
        &level->_initial_keypoints, fast_threshold, 1);

    // Keep track of this number before filtering out keyoints with the
    // grid bucketing, to use it as a signal for FAST threshold adjustment.
    level->_num_detected_keypoints = static_cast<int>(level->_initial_keypoints.size());

    if (_use_bucketing) {
      tictoc("bucketing");
      level->_grid_filter.filter(&level->_initial_keypoints);
      tictoc("bucketing");
    }

    level->_num_keypoints = 0;

    int num_kp_candidates = level->_initial_keypoints.size();

    // increase buffer size if needed
    if(num_kp_candidates > level->_keypoints_capacity) {
      level->increase_capacity((int)(num_kp_candidates * 1.2));
    }

    int num_features = 0;
    for(int kp_ind=0; kp_ind<num_kp_candidates; ++kp_ind) {

      KeyPoint& kp_cand = level->_initial_keypoints[kp_ind];

      if (!level->isLegalKeypointCoordinate(kp_cand.u, kp_cand.v)) {
        continue;
      }

      // Fill in keypoint data
      KeypointData kpdata;
      kpdata.kp = kp_cand;
      kpdata.base_uv(0) = kp_cand.u * (1 << level_num);
      kpdata.base_uv(1) = kp_cand.v * (1 << level_num);
      // lookup rectified pixel coordinates
      int pixel_index = (int)(kpdata.base_uv(1) * _base_width + kpdata.base_uv(0));
      _rectify_map->rectifyLookupByIndex(pixel_index, &kpdata.rect_base_uv);
      if(kpdata.rect_base_uv(0) < 0 || kpdata.rect_base_uv(0) >= _base_width ||
         kpdata.rect_base_uv(1) < 0 || kpdata.rect_base_uv(1) >= _base_height) {
        continue;
      }
      kpdata.pyramid_level = level_num;
      kpdata.has_depth = false;
      kpdata.xyzw = Eigen::Vector4d(NAN, NAN, NAN, NAN);
      kpdata.keypoint_index = num_features;
      kpdata.track_id = -1;

      num_features++;

      level->_keypoints[level->_num_keypoints] = kpdata;
      level->_num_keypoints++;
    }

    // keep sorted by v-coordinate, helps for filtering by epipolar line
    std::sort(&(level->_keypoints[0]),
              &(level->_keypoints[level->_num_keypoints]),
              keypoint_rect_v_comparator);

    // extract features
    level->populateDescriptorsAligned(level->_keypoints, level->_num_keypoints,
                                      level->_descriptors);

  }
}

}
