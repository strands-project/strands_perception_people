#ifndef __fovis_stereo_depth_hpp__
#define __fovis_stereo_depth_hpp__

#include <inttypes.h>

#include "camera_intrinsics.hpp"
#include "depth_source.hpp"
#include "frame.hpp"
#include "stereo_frame.hpp"
#include "feature_match.hpp"
#include "options.hpp"

#include "motion_estimation.hpp"

namespace fovis
{

/**
 * \ingroup DepthSources
 * \brief Calibration data structure for stereo cameras.
 */
struct StereoCalibrationParameters
{
  /**
   * Translation vector: [ x, y, z ]
   */
  double right_to_left_translation[3];
  /**
   * Rotation quaternion: [ w, x, y, z ]
   */
  double right_to_left_rotation[4];

  /**
   * Intrinsics of the left camera.
   */
  CameraIntrinsicsParameters left_parameters;

  /**
   * Intrinsics of the right camera.
   */
  CameraIntrinsicsParameters right_parameters;
};

/**
 * \ingroup DepthSources
 * \brief Computes useful information from a StereoCalibrationParameters object
 */
class StereoCalibration
{
  public:
    StereoCalibration(const StereoCalibrationParameters& params);
    ~StereoCalibration();

    /**
     * Compute the 4x4 transformation matrix mapping [ u, v, disparity, 1 ]
     * coordinates to [ x, y, z, w ] homogeneous coordinates in camera
     * space.
     */
    Eigen::Matrix4d getUvdToXyz() const {
      double fx_inv = 1./_rectified_parameters.fx;
      double base_inv = 1./getBaseline();
      double cx = _rectified_parameters.cx;
      double cy = _rectified_parameters.cy;
      Eigen::Matrix4d result;
      result <<
      fx_inv    , 0      , 0               , -cx * fx_inv ,
      0         , fx_inv , 0               , -cy * fx_inv ,
      0         , 0      , 0               , 1            ,
      0         , 0      , fx_inv*base_inv , 0;
      return result;
    }

    /**
     * \return the width of the rectified camera.
     */
    int getWidth() const {
      return _rectified_parameters.width;
    }

    /**
     * \return the height of the rectified camera.
     */
    int getHeight() const {
      return _rectified_parameters.height;
    }

    double getBaseline() const {
      return -_parameters.right_to_left_translation[0];
    }

    const Rectification* getLeftRectification() const {
      return _left_rectification;
    }

    const Rectification* getRightRectification() const {
      return _right_rectification;
    }

    const CameraIntrinsicsParameters& getRectifiedParameters() const {
      return _rectified_parameters;
    }

    /**
     * \return a newly allocated copy of this calibration object.
     */
    StereoCalibration* makeCopy() const;

  private:
    StereoCalibration() { }
    void initialize();

    StereoCalibrationParameters _parameters;
    CameraIntrinsicsParameters _rectified_parameters;
    Rectification* _left_rectification;
    Rectification* _right_rectification;
};

/**
 * \ingroup DepthSources
 * \brief Stores image data for a stereo camera pair.
 *
 * TODO
 */
class StereoDepth : public DepthSource
{
  public:
    StereoDepth(const StereoCalibration* calib,
                const VisualOdometryOptions& options);

    ~StereoDepth();

    void setRightImage(const uint8_t * img);

    virtual bool haveXyz(int u, int v);

    virtual void getXyz(OdometryFrame * frame);

    virtual void refineXyz(FeatureMatch * matches,
                           int num_matches,
                           OdometryFrame * frame);

    virtual double getBaseline() const { return _calib->getBaseline(); }

  private:
    typedef std::vector<std::pair<double, double> > Points2d;

    void leftRightMatch(PyramidLevel* left_level,
                        PyramidLevel* right_level,
                        Points2d* matched_right_keypoints);

    const StereoCalibration* _calib;

    int _width;
    int _height;

    int _feature_window_size;

    int _num_pyramid_levels;

    // params for adaptive feature detector threshold
    int _fast_threshold;
    int _fast_threshold_min;
    int _fast_threshold_max;
    int _target_pixels_per_feature;
    float _fast_threshold_adaptive_gain;

    bool _use_adaptive_threshold;
    bool _require_mutual_match;
    double _max_dist_epipolar_line;
    double _max_refinement_displacement;

    StereoFrame* _right_frame;

    FeatureMatcher _matcher;
    std::vector<Points2d> _matched_right_keypoints_per_level;
    std::vector<std::vector<int> > _legal_matches;

    Eigen::Matrix4d *_uvd1_to_xyz;

    int _max_disparity;

    const VisualOdometryOptions _options;

    // matches buffer.
    FeatureMatch* _matches;
    int _num_matches;
    int _matches_capacity;

};

}

#endif
