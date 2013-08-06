#ifndef __fovis_calibration_hpp__
#define __fovis_calibration_hpp__

#include <Eigen/Geometry>

namespace fovis
{

/**
 * \ingroup FovisCore
 * \brief Intrinsic parameters for a pinhole camera with plumb-bob distortion model.
 *
 */
struct CameraIntrinsicsParameters
{
  CameraIntrinsicsParameters() :
    width(0), height(0), fx(0), fy(0), cx(0), cy(0),
    k1(0), k2(0), k3(0), p1(0), p2(0)
  {}

  /**
   * \code
   * [ fx 0  cx 0 ]
   * [ 0  fy cy 0 ]
   * [ 0  0  1  0 ]
   * \endcode
   *
   * \return a 3x4 projection matrix that transforms 3D homogeneous points in
   * the camera frame to 2D homogeneous points in the image plane.
   */
  Eigen::Matrix<double, 3, 4> toProjectionMatrix() const
  {
    Eigen::Matrix<double, 3, 4> result;
    result <<
      fx,  0, cx, 0,
       0, fy, cy, 0,
       0,  0,  1, 0;
    return result;
  }

  /**
   * Image width.
   */
  int width;

  /**
   * Image height.
   */
  int height;

  /**
   * focal length along the X axis.
   */
  double fx;

  /**
   * focal length along the Y axis.  Should generally be the same as \p fx.
   */
  double fy;

  /**
   * X-coordinate of the camera center of projection / principal point.
   */
  double cx;

  /**
   * Y-coordinate of the camera center of projection / principal point.
   */
  double cy;

  /**
   * First radial distortion coefficient (r^2) for a plumb-bob distortion model.
   *
   * \sa <a href="http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html">http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html</a>
   */
  double k1;

  /**
   * Second radial distortion coefficient (r^4) for a plumb-bob distortion model.
   */
  double k2;

  /**
   * Third radial distortion coefficient (r^6) for a plumb-bob distortion model.
   */
  double k3;

  /**
   * First tangential distortion coefficient for a plumb-bob distortion model.
   */
  double p1;

  /**
   * Second tangential distortion coefficient for a plumb-bob distortion model.
   */
  double p2;
};

}

#endif
