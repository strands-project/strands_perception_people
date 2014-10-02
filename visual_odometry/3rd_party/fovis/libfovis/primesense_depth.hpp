#ifndef __fovis_primesense_depth_hpp__
#define __fovis_primesense_depth_hpp__

#include <inttypes.h>
#include <assert.h>
#include "feature_match.hpp"
#include "camera_intrinsics.hpp"
#include "rectification.hpp"
#include "depth_source.hpp"

namespace fovis
{

/**
 * \ingroup DepthSources
 * \brief Calibration data structure for Kinect / PrimeSense sensors.
 *
 * This model is taken from the ROS wiki pages.  TODO add a link here.
 * Calibration parameters for this model can be fit by following the
 * calibration procedure on the ROS wiki.
 * TODO add a link here.
 *
 *
 * PrimeSense sensors (e.g., Kinect and PrimeSense) produce a disparity map that
 * can be used to compute a dense depth map.  For a given pixel in the IR camera,
 * with pixel coordinates [u, v] and disparity d, the depth to the imaged object
 * can be described as:
 *
 * \f[
 *   z = \frac{8 * \mbox{projector\_depth\_baseline} * f}{(\mbox{shift\_offset} - d)}
 * \f]
 *
 * where \f$f\f$ is the IR camera focal length.
 *
 * The \f$x\f$ and \f$y\f$ coordinates, in the IR camera frame, can be computed as:
 *
 * \f{eqnarray*}{
 *   x &=& (u - c_x) \frac{z}{f} \\
 *   y &=& (v - c_y) \frac{z}{f}
 * \f}
 *
 * where \f$[c_x, c_y]\f$ is the center of projection for the IR camera.
 *
 * Note that this coordinate convention is defined according to standard
 * camera frames in computer graphics and vision, where +Z points along the
 * optical axis away from the camera, +X is to the right, and +y is down.
 * PrimeSense uses a different coordinate frame convention.
 */
struct PrimeSenseCalibrationParameters
{
  /**
   * image width
   */
  int width;
  /**
   * image height
   */
  int height;

  /**
   * fixed disparity offset.
   */
  double shift_offset;
  /**
   * distance from projector to depth camera (m)
   */
  double projector_depth_baseline;

  /**
   * Translation vector: [ x, y, z ]
   */
  double depth_to_rgb_translation[3];
  /**
   * Rotation quaternion: [ w, x, y, z ]
   */
  double depth_to_rgb_quaternion[4];

  /**
   * intrinsics of the IR camera.
   */
  CameraIntrinsicsParameters depth_params;
  /**
   * intrinsics of the RGB camera.
   */
  CameraIntrinsicsParameters rgb_params;
};

/**
 * \ingroup DepthSources
 * \brief Computes useful information from a PrimeSenseCalibrationParameters object
 *
 */
class PrimeSenseCalibration
{
  public:
    /**
     * Constructs a PrimeSenseCalibration object using the specified
     * parameters.
     */
    PrimeSenseCalibration(const PrimeSenseCalibrationParameters& params);
    ~PrimeSenseCalibration();

    /**
     * Compute the 4x4 transformation matrix mapping [ u, v, disparity, 1 ]
     * coordinates to [ x, y, z, w ] homogeneous coordinates in depth camera
     * space.
     */
    Eigen::Matrix4d getDepthUvdToDepthXyz() const {
      double fx_inv = 1 / params.depth_params.fx;
      double pdb_inv = 1 / params.projector_depth_baseline;
      double a = -0.125 * fx_inv * pdb_inv;
      double b = 0.125 * params.shift_offset * fx_inv * pdb_inv;
      double cx = params.depth_params.cx;
      double cy = params.depth_params.cy;
      Eigen::Matrix4d result;
      result <<
        fx_inv, 0, 0, -cx * fx_inv,
        0, fx_inv, 0, -cy * fx_inv,
        0, 0, 0, 1,
        0, 0, a, b;
      return result;
    }

    /**
     * Compute the transformation mapping [ x, y, z, w ] coordinates in
     * depth camera space to RGB camera space.
     */
    Eigen::Isometry3d getDepthXyzToRgbXyz() const {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
      result.translate(Eigen::Vector3d(params.depth_to_rgb_translation));
      const double *q = params.depth_to_rgb_quaternion;
      result.rotate(Eigen::Quaterniond(q[0], q[1], q[2], q[3]));
      return result;
    }

    /**
     * Compute the 3x4 transformation matrix projecting [ x, y, z, w ]
     * coordinates in RGB camera space to RGB image space.
     */
    Eigen::Matrix<double, 3, 4> getRgbXyzToRgbUvw() const {
      return rgb_rectification->getInputCameraParameters().toProjectionMatrix();
    }

    // ============= convenience functions =================

    /**
     * Convenience function to retrieve the transformation mapping [u, v,
     * disparity] coordinates in the depth camera space to [u, v, w] coordinates in
     * rectified RGB image space.  In most cases, the w coordinate will not be 1,
     * so to normalize the [u, v, w] homogeneous coordinate to pixel coordinates, u
     * and v need to be divided by w.
     */
    Eigen::Matrix<double, 3, 4> getDepthUvdToRgbUvw() const {
      return getRgbXyzToRgbUvw() * getDepthXyzToRgbXyz().matrix() * getDepthUvdToDepthXyz();
    }

    /**
     * Convenience function to retrieve the transformation mapping [u, v,
     * disparity] coordinates in the depth camera space to [X, Y, Z, W] coordinates in
     * RGB XYZ space.
     */
    Eigen::Matrix4d getDepthUvdToRgbXyz() const {
      return getDepthXyzToRgbXyz().matrix() * getDepthUvdToDepthXyz();
    }

    /**
     * \return the calibration parameters used to initialize this object.
     */
    const PrimeSenseCalibrationParameters& getParameters() const {
      return params;
    }

    /**
     * return the rectification object calculated from the calibration parameters.
     */
    const Rectification* getRgbRectification() const {
      return rgb_rectification;
    }

  private:
    PrimeSenseCalibrationParameters params;

    Rectification* rgb_rectification;
};

/**
 * \ingroup DepthSources
 * \brief Stores depth data for a Kinect / PrimeSense camera.
 *
 * \sa PrimeSenseCalibrationParameters, PrimeSenseCalibration
 */
class PrimeSenseDepth : public DepthSource
{
  public:
    PrimeSenseDepth(const PrimeSenseCalibration* calib);
    ~PrimeSenseDepth();

    void setDisparityData(const uint16_t* disparity);

    bool getXyz(int u, int v, Eigen::Vector3d & xyz);
    void getXyz(OdometryFrame * frame);
    void refineXyz(FeatureMatch * matches,
                   int num_matches,
                   OdometryFrame * frame);

    /**
     * Retrieve the disparity value at the IR pixel that projects on to the
     * specified RGB pixel.
     *
     * Disparity is not measured on the RGB image directly.  Instead, it's
     * measured on the IR image, then the points are projected onto the RGB
     * image.  This function looks up the IR pixel that projected onto the
     * specified RGB pixel, and then returns the disparity of the IR pixel.
     *
     * Returns: the measured disparity, or 0 if there is no disparity data for
     * the specified pixel.
     */
    inline uint16_t getDisparity(int rgb_u, int rgb_v) const;

    /**
     * Retrieve the disparity value at the specified RGB pixel.
     */
    inline uint16_t getDisparityAtIRPixel(int ir_u, int ir_v) const;

    inline double getBaseline() const;

    inline bool haveXyz(int u, int v);

  private:
    bool haveXyz(float u_f, float v_f);

    bool getXyzFast(KeypointData* kpdata);
    bool getXyzInterp(KeypointData* kpdata);

    int _ir_width;
    int _ir_height;

    const PrimeSenseCalibration* _calib;

    uint16_t* _disparity;

    // for each RGB camera pixel, stores the index of the corresponding
    // disparity image pixel (v*width + u)
    uint32_t* _rgb_to_disparity_map;

    uint32_t _disparity_map_max_index;

    Eigen::Matrix4d* _uvd1_d_to_xyz_c;
};

uint16_t
PrimeSenseDepth::getDisparity(int u, int v) const
{
  int rgb_index = v * _ir_width + u;
  uint32_t disparity_index = _rgb_to_disparity_map[rgb_index];
  if(disparity_index < _disparity_map_max_index) {
    return _disparity[disparity_index];
  } else {
    return 0;
  }
}

uint16_t
PrimeSenseDepth::getDisparityAtIRPixel(int ir_u, int ir_v) const
{
  assert(ir_u >= 0 && ir_v >= 0 && ir_u < _ir_width && ir_v < _ir_height);
  return _disparity[ir_v * _ir_width + ir_u];
}

double
PrimeSenseDepth::getBaseline() const
{
  return _calib->getParameters().projector_depth_baseline;
}

bool
PrimeSenseDepth::haveXyz(int u, int v)
{
  int rgb_index = v * _ir_width + u;
  uint32_t disparity_index = _rgb_to_disparity_map[rgb_index];
  return disparity_index < _disparity_map_max_index;
}

}
#endif
