#ifndef __fovis_rectification_hpp__
#define __fovis_rectification_hpp__

#include <inttypes.h>

#include <Eigen/Core>

#include "camera_intrinsics.hpp"

namespace fovis
{

/**
 * \ingroup FovisCore
 * \brief Maps image coordinates from an input image to image coordinates on a
 * rectified camera.
 *
 * A Rectification object represents the computation required to map pixels
 * from an input camera to pixels on a "rectified" virtual camera that shares
 * the same focal point, but may be rotated and have different projection
 * parameters (e.g., focal length, center of projection, etc.)
 *
 * This class is primary useful with stereo cameras to rectify pixels on both
 * input cameras to rectified cameras that share an image plane and look/up
 * vectors.  It can also be used for simple undistortion (e.g., no rotation,
 * and a rectified camera that has the same projection parameters as the input
 * camera, but with no distortion).
 *
 * For fast rectification, the Rectification object precomputes a mapping from
 * (u, v) image pixel coordinates to u', v' rectified image pixel coordinates.
 * The exact transformation from (u, v) to (u', v') is:
 *
 * -# undistort according to a plumb-bob distortion model
 * -# rotate points about the origin
 * -# reproject onto a rectified image plane.
 *
 * Rectification of pixels is done by bilinear interpolation on this
 * precomputed mapping.
 *
 */
class Rectification
{
  public:

    /**
     * Convenience constructor to create a Rectification object that simply
     * undistorts.  The rotation is set to identity, and the rectified camera
     * parameters are identical to the input camera parameters, but have no
     * distortion.
     */
    Rectification(const CameraIntrinsicsParameters& input_camera_params);

    /**
     * Constructor.  The rectified_camera_params are not allowed to have any
     * distortion.
     */
    Rectification(const CameraIntrinsicsParameters& input_camera_params,
            const Eigen::Matrix3d& rotation,
            const CameraIntrinsicsParameters& rectified_camera_params);

    ~Rectification();

    /**
     * \return the intrisic parameters of the input camera.
     */
    const CameraIntrinsicsParameters& getInputCameraParameters() const {
      return _input_camera;
    }

    /**
     * \return the rotation to apply between the undistorted input camera frame
     * and the output rectified camera frame.
     */
    const Eigen::Matrix3d& getRectificationRotation() const {
      return *_rotation;
    }

    /**
     * \return the projection parameters of the rectified camera.  The
     * distortion coefficients will always be zero.  Note that the focal length
     * and image dimensions of the rectified camera do not have to match those
     * of the input camera.
     */
    const CameraIntrinsicsParameters& getRectifiedCameraParameters() const {
      return _rectified_camera;
    }

    /**
     * Computes the undistorted image coordinates of the input distorted
     * coordinates (\p dist_u, \p dist_v).
     *
     * \param dist_u input distorted pixel u/x coordinate.
     * \param dist_v input distorted pixel v/y coordinate.
     * \param rect_uv output parameter.
     */
    void rectifyLookup(int dist_u, int dist_v,
                       Eigen::Vector2d* rect_uv) const
    {
      assert(dist_u >= 0 && dist_v >= 0 && dist_u < _input_camera.width && dist_v < _input_camera.height);
      int pixel_index = dist_v * _input_camera.width + dist_u;
      rect_uv->x() = _map_x[pixel_index];
      rect_uv->y() = _map_y[pixel_index];
    }

    /**
     * Computes the undistorted image coordinates of an input pixel specified
     * by its row-major pixel index.
     *
     * \param pixel_index corresponds to \f$ u * width + v \f$
     * \param rect_uv output parameter.
     *
     * \sa rectifyLookup
     */
    void rectifyLookupByIndex(int pixel_index,
                              Eigen::Vector2d* rect_uv) const
    {
      rect_uv->x() = _map_x[pixel_index];
      rect_uv->y() = _map_y[pixel_index];
    }

    /**
     * Computes the undistorted image coordinates of an input pixel via
     * bilinear interpolation of its integer-coordinate 4-neighboors.
     *
     * \param dist_uv input distorted pixel coordinates.
     * \param rect_uv output parameter.
     */
    void rectifyBilinearLookup(const Eigen::Vector2d& dist_uv,
                               Eigen::Vector2d* rect_uv) const {
      int u = (int)dist_uv.x();
      int v = (int)dist_uv.y();

      assert(u >= 0 && v >= 0 && u < _input_camera.width && v < _input_camera.height);

      // weights
      float wright  = (dist_uv.x() - u);
      float wbottom = (dist_uv.y() - v);
      float w[4] = {
        (1 - wright) * (1 - wbottom),
        wright * (1 - wbottom),
        (1 - wright) * wbottom,
        wright * wbottom
      };

      int ra_index = v * _input_camera.width + u;
      uint32_t neighbor_indices[4] = {
        ra_index,
        ra_index + 1,
        ra_index + _input_camera.width,
        ra_index + _input_camera.width + 1
      };

      rect_uv->x() = 0;
      rect_uv->y() = 0;
      for(int i = 0; i < 4; ++i) {
        rect_uv->x() += w[i] * _map_x[neighbor_indices[i]];
        rect_uv->y() += w[i] * _map_y[neighbor_indices[i]];
      }
    }

    /**
     * \return a deep copy of this object.
     */
    Rectification* makeCopy() const;

  private:
    Rectification() {}

    void populateMap();

    CameraIntrinsicsParameters _input_camera;

    Eigen::Matrix3d* _rotation;
    // use a pointer above to avoid EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CameraIntrinsicsParameters _rectified_camera;

    float* _map_x;
    float* _map_y;
};

} /*  */

#endif
