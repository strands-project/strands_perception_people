#ifndef __fovis_pyramid_level_hpp__
#define __fovis_pyramid_level_hpp__

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "keypoint.hpp"
#include "intensity_descriptor.hpp"
#include "grid_filter.hpp"

namespace fovis
{

/**
 * \ingroup FovisCore
 * \brief One level of a Gaussian image pyramid.
 *
 * TODO
 */
class PyramidLevel
{
  public:
    PyramidLevel(int width, int height, int level_num, int feature_window_size,
                 GridKeyPointFilter& grid_filter);
    ~PyramidLevel();

    const uint8_t* getGrayscaleImage() const {
      return _raw_gray;
    }

    int getGrayscaleImageStride() const {
      return _raw_gray_stride;
    }

    const uint8_t* getDescriptor(int i) const {
      return _descriptors + i * getDescriptorStride();
    }


    int getDescriptorStride() const {
      return _descriptor_extractor->getDescriptorStride();
    }

    int getNumKeypoints() const {
      return _num_keypoints;
    }

    int getDescriptorLength() const {
      return _descriptor_extractor->getDescriptorLength();
    }

    const KeyPoint& getKeypoint(int i) const {
      return _keypoints[i].kp;
    }

    const Eigen::Vector4d& getKeypointXYZW(int i) const {
      return _keypoints[i].xyzw;
    }

    const float getKeypointRectBaseU(int kp_index) const {
      return _keypoints[kp_index].rect_base_uv(0);
    }

    const float getKeypointRectBaseV(int kp_index) const {
      return _keypoints[kp_index].rect_base_uv(1);
    }

    const Eigen::Vector2d& getKeypointRectBaseUV(int kp_index) const {
      return _keypoints[kp_index].rect_base_uv;
    }

    const KeypointData* getKeypointData(int i) const {
        return &_keypoints[i];
    }

    KeypointData* getKeypointData(int i) {
        return &_keypoints[i];
    }

    int getLevelNum() const { return _level_num; }

    void populateDescriptorInterp(float x, float y, uint8_t* descriptor) const;

    void populateDescriptorAligned(int x, int y, uint8_t* descriptor) const;

    void populateDescriptorsInterp(const KeypointData* keypoints,
                                    int num_keypoints,
                                    uint8_t* descriptors) const;

    void populateDescriptorsAligned(const KeypointData* keypoints,
                                    int num_keypoints,
                                    uint8_t* descriptors) const;

    int getWidth() const { return _width; }
    int getHeight() const { return _height; }

    bool isLegalKeypointCoordinate(float x, float y) const {
      return x >= _keypoint_min_x && x <= _keypoint_max_x &&
             y >= _keypoint_min_y && y <= _keypoint_max_y;
    }

    const int* getDescriptorIndexOffsets() const {
      return _descriptor_extractor->getDescriptorIndexOffsets();
    }

    const std::vector<KeyPoint>& getInitialFeatures() const {
      return _initial_keypoints;
    }

    int getNumDetectedKeypoints() const {
      return _num_detected_keypoints;
    }

  private:
    friend class OdometryFrame;
    friend class StereoFrame;

    void increase_capacity(int new_capacity);

    uint8_t* _raw_gray;
    int _raw_gray_stride;

    std::vector<KeyPoint> _initial_keypoints;
    int _num_detected_keypoints;

    GridKeyPointFilter _grid_filter;

    KeypointData* _keypoints;
    int _num_keypoints;
    int _keypoints_capacity;

    uint8_t* _descriptors;

    int _keypoint_min_x;
    int _keypoint_min_y;
    int _keypoint_max_x;
    int _keypoint_max_y;

    int _width;
    int _height;
    int _level_num;

    uint8_t* _pyrbuf;

    IntensityDescriptorExtractor * _descriptor_extractor;
};

}
#endif
