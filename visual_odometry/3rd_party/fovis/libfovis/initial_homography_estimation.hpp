#ifndef __fovis_initial_homography_estimator_hpp__
#define __fovis_initial_homography_estimator_hpp__

#include <vector>
#include <stdint.h>
#include <Eigen/Dense>

namespace fovis
{

/**
 * \brief Estimates a rough 2D homography registering two images.
 */
class InitialHomographyEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  /**
   * Set the template image to the passed in arguments.
   * assumes the image data is row-major.
   * The image will be downsampled by \f$ 1/2^{downsampleFactor} \f$
   */
  void setTemplateImage(const uint8_t * grayData, int width, int height, int stride, int downsampleFactor);

  /**
   * Set the test image accordingly. The opimization will warp this image to
   * match the template assumes the image data is row-major.
   * The image will be downsampled by \f$ 1/2^{downsampleFactor} \f$
   */
  void setTestImage(const uint8_t * grayData, int width, int height, int stride, int downsampleFactor);

  /**
   * Run ESM to find the homography between the template and test images. These
   * should have already been passed in using the methods setTemplateImage(), and
   * setTestImage().
   */
  Eigen::Matrix3f track(const Eigen::Matrix3f &init_H, int nIters, double *finalRMS);

private:
  int template_rows, template_cols; //size of the template
  Eigen::MatrixXf templateImage, testImage; //the images
  Eigen::MatrixXf warpedTestImage; //storage for the warped testImage on the current iteration
  Eigen::MatrixXf errorIm; //the difference between the templateImage and the warpedTestImage
  Eigen::ArrayXf templateDxRow, templateDyRow; //the gradient of the template flattened into a row vector
  Eigen::MatrixXf templatePoints; //nx3 matrix storing the x,y,1 for each pixel
  Eigen::ArrayXf xx, yy; //convenience array storing the original x/y for each pixel in the flattened image

  //internal functions

  /**
   * Compute the x/y gradient of the passed in image
   */
  static void computeGradient(const Eigen::MatrixXf &image, Eigen::MatrixXf * dxp, Eigen::MatrixXf *dyp);

  /**
   * Compute the error (RMS) for the passed in difference image
   */
  static double computeError(const Eigen::MatrixXf &error);

  /**
   * form the Jacobian
   */
  Eigen::MatrixXf computeJacobian(const Eigen::ArrayXf &dx, const Eigen::ArrayXf &dy) const;

  /**
   * Compute the homography from the lie parameterization
   */
  Eigen::Matrix3f lieToH(const Eigen::VectorXf &lie) const;

  /**
   * Warp srcImage according to the warped points
   */
  Eigen::MatrixXf constructWarpedImage(const Eigen::MatrixXf &srcImage, const Eigen::MatrixXf &warpedPoints) const;

  /**
   * Flatten an image stored as a matrix down into a row vector
   */
  static Eigen::ArrayXf flattenMatrix(Eigen::MatrixXf &m);

};

}

#endif
