#include <stdio.h>

#include <iostream>

#include <emmintrin.h>

#include "visual_odometry.hpp"
#include "frame.hpp"
#include "fast.hpp"
#include "gauss_pyramid.h"
#include "camera_intrinsics.hpp"
#include "rectification.hpp"
#include "depth_source.hpp"
#include "internal_utils.hpp"
#include "normalize_image.hpp"

#include "tictoc.hpp"

#ifndef ALIGNMENT
#define ALIGNMENT 16
#endif

using namespace std;

// ========================= OdometryFrame =========================

namespace fovis
{

OdometryFrame::OdometryFrame(const Rectification* rectification,
                             const VisualOdometryOptions& options)
{
  const CameraIntrinsicsParameters& input_camera = rectification->getInputCameraParameters();
  _rectification = rectification;
  _orig_width = input_camera.width;
  _orig_height = input_camera.height;


  const VisualOdometryOptions& defaults(VisualOdometry::getDefaultOptions());
  _num_levels = optionsGetIntOrFromDefault(options, "max-pyramid-level", defaults);
  _feature_window_size = optionsGetIntOrFromDefault(options, "feature-window-size", defaults);
  _use_image_normalization = optionsGetBoolOrFromDefault(options, "use-image-normalization", defaults);

  _use_bucketing = optionsGetBoolOrFromDefault(options, "use-bucketing", defaults);
  int bucket_width = optionsGetIntOrFromDefault(options, "bucket-width", defaults);
  int bucket_height = optionsGetIntOrFromDefault(options, "bucket-height", defaults);
  int max_keypoints_per_bucket = optionsGetIntOrFromDefault(options, "max-keypoints-per-bucket", defaults);

  for (int level_num=0; level_num<_num_levels; level_num++) {
    int level_width = _orig_width >> level_num;
    int level_height = _orig_height >> level_num;
    // hard code parameters for now
    GridKeyPointFilter grid_filter(level_width, level_height, bucket_width,
                                   bucket_height, max_keypoints_per_bucket);

    PyramidLevel* level = new PyramidLevel(level_width, level_height,
                                           level_num, _feature_window_size,
                                           grid_filter);
    _levels.push_back(level);
  }
}

OdometryFrame::~OdometryFrame()
{
  for (unsigned i=0; i<_levels.size(); i++)
    delete _levels[i];
  _levels.clear();
  _num_levels = 0;
}

void
OdometryFrame::prepareFrame(const uint8_t* raw_gray,
                            int fast_threshold,
                            DepthSource* depth_source)
{
  // copy raw image to first pyramid level
  assert(_num_levels);
  PyramidLevel* first_level = _levels[0];
  int src_stride = _orig_width;
  for (int row=0; row<_orig_height; row++) {
    memcpy(first_level->_raw_gray + row * first_level->_raw_gray_stride,
           raw_gray + row * src_stride, _orig_width);
  }

  if (_use_image_normalization) {
    normalize_image(first_level->_raw_gray, first_level->_raw_gray_stride,
                    _orig_width, _orig_height);

  }

  // compute image pyramid and detect initial features
  for (int level_num=0; level_num<_num_levels; level_num++) {
    PyramidLevel* level = _levels[level_num];

    if (level_num > 0) {
      // resize the image from the previous level
      PyramidLevel* prev_level = _levels[level_num-1];
      int prev_width = prev_level->getWidth();
      int prev_height = prev_level->getHeight();
      gauss_pyr_down_8u_C1R(prev_level->_raw_gray, prev_level->_raw_gray_stride,
          prev_width, prev_height,
          level->_raw_gray, level->_raw_gray_stride,
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
    if (num_kp_candidates > level->_keypoints_capacity) {
      level->increase_capacity(static_cast<int>(num_kp_candidates*1.2));
    }

    int min_dist_from_edge = (_feature_window_size - 1) / 2 + 1;
    int min_x = min_dist_from_edge;
    int min_y = min_dist_from_edge;
    int max_x = level->_width - (min_dist_from_edge + 1);
    int max_y = level->_height - (min_dist_from_edge + 1);

    // filter the keypoint candidates, and compute derived data
    for (int kp_ind=0; kp_ind<num_kp_candidates; kp_ind++) {
      KeyPoint& kp_cand = level->_initial_keypoints[kp_ind];

      // ignore features too close to border
      if(kp_cand.u < min_x || kp_cand.u > max_x || kp_cand.v < min_y ||
         kp_cand.v > max_y)
        continue;

      KeypointData kpdata;
      kpdata.kp = kp_cand;
      kpdata.base_uv(0) = kp_cand.u * (1 << level_num);
      kpdata.base_uv(1) = kp_cand.v * (1 << level_num);
      kpdata.pyramid_level = level_num;

      assert(kpdata.base_uv(0) >= 0);
      assert(kpdata.base_uv(1) < _orig_width);
      assert(kpdata.base_uv(0) >= 0);
      assert(kpdata.base_uv(1) < _orig_height);

      // lookup rectified pixel coordinates
      int pixel_index = static_cast<int>(kpdata.base_uv(1) * _orig_width + kpdata.base_uv(0));
      _rectification->rectifyLookupByIndex(pixel_index, &kpdata.rect_base_uv);

      // Ignore the points that fall
      // outside the original image region when undistorted.
      if (kpdata.rect_base_uv(0) < 0 || kpdata.rect_base_uv(0) >= _orig_width ||
          kpdata.rect_base_uv(1) < 0 || kpdata.rect_base_uv(1) >= _orig_height) {
        continue;
      }

      // ignore features with unknown depth
      int du = static_cast<int>(kpdata.rect_base_uv(0)+0.5);
      int dv = static_cast<int>(kpdata.rect_base_uv(1)+0.5);
      if (!depth_source->haveXyz(du, dv)) { continue; }

      // We will calculate depth of all the keypoints later
      kpdata.xyzw = Eigen::Vector4d(NAN, NAN, NAN, NAN);
      kpdata.has_depth = false;
      kpdata.keypoint_index = level->_num_keypoints;

      kpdata.track_id = -1; //hasn't been associated with a track yet

      level->_keypoints[level->_num_keypoints] = kpdata;
      level->_num_keypoints++;
    }

    // extract features
    level->populateDescriptorsAligned(level->_keypoints, level->_num_keypoints,
                                      level->_descriptors);
  }

  // populate 3D position for descriptors. Depth calculation may fail for some
  // of these keypoints.
  depth_source->getXyz(this);

  // Get rid of keypoints with no depth.
  purgeBadKeypoints();

}

/**
 * Delete keypoints (and corresponding descriptors) for which we couldn't get depth.
 */
void
OdometryFrame::purgeBadKeypoints() {
  tictoc("purge_bad_keypoints");
  for(int level_num=0; level_num<_num_levels; ++level_num) {
    PyramidLevel* level = _levels[level_num];
    int desc_stride = level->getDescriptorStride();
    // Invariant: kp_data_end and descriptors_end point at end
    // of arrays with 'good' keypoints and descriptors respectively
    KeypointData* kp_data_end = &(level->_keypoints[0]);
    uint8_t* descriptors_end = &(level->_descriptors[0]);
    for (int kp_ind=0; kp_ind < level->_num_keypoints; ++kp_ind) {
      KeypointData * kp_data = &(level->_keypoints[kp_ind]);
      uint8_t *desc = level->_descriptors + kp_ind * desc_stride;
      if (kp_data->has_depth) {
        // Keep this keypoint - copy it to end of 'good' keypoint array
        // Avoid redundant copying
        if (kp_data_end != kp_data) {
          *kp_data_end = *kp_data;
          memcpy(descriptors_end, desc, desc_stride);
        }
        ++kp_data_end;
        descriptors_end += desc_stride;
      }
    }
    int new_num_keypoints = kp_data_end - &(level->_keypoints[0]);
    level->_num_keypoints = new_num_keypoints;
    // Re-index keypoints to keep things consistent
    for (int kp_ind=0; kp_ind < level->_num_keypoints; ++kp_ind) {
      KeypointData& kp_data(level->_keypoints[kp_ind]);
      assert (kp_data.has_depth);
      kp_data.keypoint_index = kp_ind;
    }
  }
  tictoc("purge_bad_keypoints");
}

void
OdometryFrame::sanityCheck() const
{
#ifndef NDEBUG
  for(int level_ind=0; level_ind<_num_levels; level_ind++) {
    const PyramidLevel* level = _levels[level_ind];
    int num_keypoints = level->_num_keypoints;
    for(int f_ind=0; f_ind<num_keypoints; f_ind++) {
      const KeypointData& kp = level->_keypoints[f_ind];

      assert(kp.kp.u >= 0 && kp.kp.v >= 0 &&
          kp.kp.u < level->_width && kp.kp.v < level->_height);
    }
  }
#endif
}

}
