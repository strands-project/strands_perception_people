#ifndef __fovis_frame_hpp__
#define __fovis_frame_hpp__

#include <stdint.h>
#ifndef NDEBUG
#include <assert.h>
#endif

#include <vector>
#include <Eigen/Geometry>

#include "options.hpp"
#include "keypoint.hpp"
#include "grid_filter.hpp"
#include "pyramid_level.hpp"

namespace fovis
{

class CameraIntrinsics;
class Rectification;
class DepthSource;

/**
 * @ingroup FovisCore
 * @brief Stores data specific to an input frame.
 * @include: fovis/fovis.hpp
 *
 * Stores grayscaled image, disparity map, keypoints (detected features),
 * and keypoint descriptors.
 *
 */
class OdometryFrame
{
  public:
    OdometryFrame(const Rectification* rectification,
                  const VisualOdometryOptions& options);

    ~OdometryFrame();

    void prepareFrame(const uint8_t* raw_gray,
        int fast_threshold,
        DepthSource* depth_source);

    int getNumKeypoints() const {
      int result = 0;
      for(int i=0; i<_num_levels; i++)
        result += _levels[i]->getNumKeypoints();
      return result;
    }

    int getNumDetectedKeypoints() const {
      int result = 0;
      for(int i=0; i<_num_levels; i++)
        result += _levels[i]->getNumDetectedKeypoints();
      return result;
    }

    int getNumLevels() const {
      return _num_levels;
    }

    const PyramidLevel* getLevel(int i) const {
      return _levels[i];
    }

    PyramidLevel* getLevel(int i) {
      return _levels[i];
    }

    void sanityCheck() const;

    int getFeatureWindowSize() const { return _feature_window_size; }

  private:

    void purgeBadKeypoints();

    int _orig_width;
    int _orig_height;

    int _num_levels;
    int _feature_window_size;
    bool _use_bucketing;
    bool _use_image_normalization;

    // note: the rectification pointer is 'borrowed'
    const Rectification* _rectification;

    std::vector<PyramidLevel*> _levels;
};

}

#endif
