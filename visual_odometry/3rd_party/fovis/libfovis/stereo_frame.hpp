#ifndef __fovis_stereo_frame_hpp__
#define __fovis_stereo_frame_hpp__

#include <stdint.h>
#include <assert.h>

#include <vector>
#include <Eigen/Geometry>

#include "options.hpp"
#include "keypoint.hpp"
#include "pyramid_level.hpp"
#include "rectification.hpp"

namespace fovis
{

/**
 * \ingroup DepthSources
 * \brief Stores the right-hand image for a stereo pair.
 *
 * TODO
 */
class StereoFrame
{
  public:
    StereoFrame(int width, int height,
                const Rectification* rectify_map,
                const VisualOdometryOptions& options);

    ~StereoFrame();

    void prepareFrame(const uint8_t* raw_gray, int fast_threshold);

    int getNumDetectedKeypoints() const {
      int result = 0;
      for(int i=0; i<_num_levels; i++)
        result += _levels[i]->getNumDetectedKeypoints();
      return result;
    }

    PyramidLevel * getLevel(int level_num) { return _levels[level_num]; }

    void rectify(Eigen::Vector2d xy_in, Eigen::Vector2d * out) {
      _rectify_map->rectifyBilinearLookup(xy_in, out);
    }

  private:
    void initialize(int bucket_width, int bucket_height, int max_keypoints_per_bucket);

    int _base_width;
    int _base_height;
    int _feature_window_size;

    int _num_levels;
    std::vector<PyramidLevel *> _levels;

    Rectification* _rectify_map;

    bool _use_bucketing;
    bool _use_image_normalization;
};

}

#endif
