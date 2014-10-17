#include <vector>
#include <iostream>
#include <iomanip>

#include <Eigen/SVD>
#include <unsupported/Eigen/MatrixFunctions>

#include "tictoc.hpp"
#include "initial_homography_estimation.hpp"

using namespace std;
using namespace Eigen;

namespace fovis
{

#define dump(var) //(cerr<<" "#var<<" =[\n"<< setprecision (12)<<var<<"];"<<endl)
Eigen::ArrayXf InitialHomographyEstimator::flattenMatrix(Eigen::MatrixXf &m)
{
  return Eigen::Map<Eigen::ArrayXf>(m.data(), m.rows() * m.cols());
}

static void 
grayToEigen(const uint8_t * grayData, int width, int height, int stride,
    int downsampleFactor, Eigen::MatrixXf* result)
{
  Eigen::MatrixXf& eig_imf = *result;
  if (downsampleFactor > 0) {
    int cols = width >> downsampleFactor;
    int rows = height >> downsampleFactor;
    eig_imf = Eigen::MatrixXf::Zero(rows, cols);
    for (int y = 0; y < height; y++) {
      int ey = y >> downsampleFactor;
      for (int x = 0; x < width; x++) {
        int ex = x >> downsampleFactor;
        eig_imf(ey, ex) += grayData[y * stride + x];
      }
    }
    double pixelFactor = (1 << downsampleFactor);
    eig_imf /= pixelFactor * pixelFactor;
  }
  else {
    eig_imf.resize(height, width);
    const uint8_t* row_start = grayData;
    for(int row=0; row<height; row++) {
      for(int col=0; col<width; col++) {
        eig_imf(row, col) = row_start[col];
      }
      row_start += stride;
    }
  }
}

void InitialHomographyEstimator::setTestImage(const uint8_t * grayData, int width, int height, int stride, int downsampleFactor)
{
  grayToEigen(grayData, width, height, stride, downsampleFactor, &testImage);
}

void InitialHomographyEstimator::setTemplateImage(const uint8_t * grayData, int width, int height, int stride,
    int downsampleFactor)
{
  grayToEigen(grayData, width, height, stride, downsampleFactor, &templateImage);
  template_rows = templateImage.rows();
  template_cols = templateImage.cols();

  //compute template gradients
  Eigen::MatrixXf templateDx, templateDy;
  computeGradient(templateImage, &templateDx, &templateDy);

  templateDxRow = flattenMatrix(templateDx);
  templateDyRow = flattenMatrix(templateDy);

  //setup the utility matrices
  Eigen::MatrixXf x = VectorXf::LinSpaced(template_cols, 0, template_cols - 1).transpose().replicate(template_rows, 1);
  Eigen::MatrixXf y = VectorXf::LinSpaced(template_rows, 0, template_rows - 1).replicate(1, template_cols);
  xx = flattenMatrix(x);
  yy = flattenMatrix(y);

  templatePoints.resize(3, xx.rows());
  templatePoints.row(0) = xx;
  templatePoints.row(1) = yy;
  templatePoints.row(2).setOnes();

  xx += 1;
  yy += 1;

}

double InitialHomographyEstimator::computeError(const Eigen::MatrixXf &error)
{
  return error.norm() / sqrt((double) error.rows() * error.cols());
}

Eigen::Matrix3f InitialHomographyEstimator::track(const Eigen::Matrix3f & initH, int nIters, double * finalRMS)
{

  double minError = INFINITY;
  Eigen::Matrix3f H = initH;
  Eigen::Matrix3f bestH = H;
  int bestIter = 0;
  int lastImproved = 0;
  double lastRMS = INFINITY;

  for (int iter = 0; iter < nIters; iter++) {
    tictoc("track_iter");
    tictoc("warpPoints");
    Eigen::MatrixXf warpedHomogeneousPoints;
    warpedHomogeneousPoints = H * templatePoints;
    tictoc("warpPoints");

    tictoc("constructWarpedImage");
    warpedTestImage = constructWarpedImage(testImage, warpedHomogeneousPoints);
    tictoc("constructWarpedImage");

    errorIm = warpedTestImage - templateImage;
    Eigen::VectorXf errorRow;
    errorRow = flattenMatrix(errorIm);

    double rmsError = computeError(errorRow);

    if (rmsError < minError) {
      minError = rmsError;
      bestH = H;
      bestIter = iter;
    }

    tictoc("computeJacobian");
    tictoc("computeGradient");
    Eigen::MatrixXf warpedTestImageDx, warpedTestImageDy;
    computeGradient(warpedTestImage, &warpedTestImageDx, &warpedTestImageDy);
    tictoc("computeGradient");

    Eigen::ArrayXf warpedTestImageDxRow, warpedTestImageDyRow;
    warpedTestImageDxRow = flattenMatrix(warpedTestImageDx);
    warpedTestImageDyRow = flattenMatrix(warpedTestImageDy);

    Eigen::MatrixXf Jt = computeJacobian(templateDxRow + warpedTestImageDxRow, templateDyRow + warpedTestImageDyRow);
    tictoc("computeJacobian");

    //compute the psuedo-inverse
    tictoc("update");
    tictoc("svd_pinv");
    Eigen::JacobiSVD<MatrixXf> svd(Jt, ComputeThinU | ComputeThinV);
    Eigen::VectorXf sigma = svd.singularValues();
    Eigen::MatrixXf U = svd.matrixU();
    Eigen::MatrixXf V = svd.matrixV();
    int r = 0;
    for (r = 0; r < sigma.rows(); r++) { //singular values are in decreasing order
      if (sigma(r) < 1e-7) //TODO:better way to get the tolerance?
        break;
      else
        sigma(r) = 1.0 / sigma(r);
    }
    Eigen::MatrixXf Jt_plus;
    if (r == 0)
      Jt_plus = Eigen::MatrixXf::Zero(Jt.cols(), Jt.rows());
    else {
      Jt_plus = V.block(0, 0, V.rows(), r) * sigma.head(r).asDiagonal() * U.block(0, 0, U.rows(), r).transpose();
    }
    tictoc("svd_pinv");

    // this doesn't seem to work :-/
    //    tictoc("manual_pinv");
    //    Eigen::Matrix3f JtT_Jt = Jt.transpose() * Jt;
    //    Eigen::MatrixXf Jt_plus = JtT_Jt.inverse() * Jt;
    //    tictoc("manual_pinv");

    Eigen::VectorXf lie_d = -2 * Jt_plus * errorRow;
    tictoc("update");

    tictoc("lieToH");
    H = H * lieToH(lie_d);
    tictoc("lieToH");

    if (rmsError < lastRMS)
      lastImproved = iter;

    tictoc("track_iter");

    //        cout << iter << ") rmsError= " << rmsError << " minError = " << minError << " d.norm() =" << lie_d.norm() << endl;
    //    exit(1);
    if (lie_d.norm() < 1e-6 || (rmsError - minError > 3 && iter - bestIter > 2) || iter - bestIter > 4 || iter
        - lastImproved > 2) {
      //      printf("breaking after %d iters\n", iter);
      break;
    }
    lastRMS = rmsError;

  }
  if (finalRMS != NULL)
    *finalRMS = minError;
  return bestH;

}

void InitialHomographyEstimator::computeGradient(const Eigen::MatrixXf &image, Eigen::MatrixXf *dxp, Eigen::MatrixXf *dyp)
{
  Eigen::MatrixXf & dx = *dxp;
  Eigen::MatrixXf & dy = *dyp;
  dx = Eigen::MatrixXf::Zero(image.rows(), image.cols());
  dy = Eigen::MatrixXf::Zero(image.rows(), image.cols());

  dx.block(0, 1, dx.rows(), dx.cols() - 2) = image.block(0, 2, dx.rows(), dx.cols() - 2) - image.block(0, 0, dx.rows(),
      dx.cols() - 2);
  //handle border

  dy.block(1, 0, dy.rows() - 2, dy.cols()) = image.block(2, 0, dy.rows() - 2, dy.cols()) - image.block(0, 0, dy.rows()
      - 2, dy.cols());
  //normalize
  dx /= 2.0;
  dy /= 2.0;

  //handle borders
  dx.col(0) = image.col(1) - image.col(0);
  dx.col(image.cols() - 1) = image.col(image.cols() - 1) - image.col(image.cols() - 2);
  dy.row(0) = image.row(1) - image.row(0);
  dy.row(image.rows() - 1) = image.row(image.rows() - 1) - image.row(image.rows() - 2);

}

Eigen::MatrixXf InitialHomographyEstimator::computeJacobian(const Eigen::ArrayXf &dx, const Eigen::ArrayXf &dy) const
{
  Eigen::MatrixXf Jt(dx.rows(), 3);
  Jt.col(0) = dx;
  Jt.col(1) = dy;
  Jt.col(2) = dx * yy - dy * xx;
  return Jt;
}

Eigen::Matrix3f InitialHomographyEstimator::lieToH(const Eigen::VectorXf &lie) const
{
  //TODO: support more parameters?
  Eigen::Matrix3f M;
  M << 0, lie(2), lie(0),
      -lie(2), 0, lie(1),
       0, 0, 0;
  return M.exp();
}

Eigen::MatrixXf InitialHomographyEstimator::constructWarpedImage(const Eigen::MatrixXf &srcImage,
    const Eigen::MatrixXf &warpedPoints) const
{
  Eigen::MatrixXf warped = Eigen::MatrixXf(template_rows, template_cols);

  const double defaultValue = 128;
  //Bilinear interpolation
  for (int i = 0; i < warpedPoints.cols(); i++) {
    double val;
    Eigen::Vector2f pt = warpedPoints.col(i).head(2) / warpedPoints(2, i);
    Eigen::Vector2i fipt(floor(pt(0)), floor(pt(1)));
    Eigen::Vector2i cipt(ceil(pt(0)), ceil(pt(1)));
    if (0 <= pt(0) && pt(0) < srcImage.cols() - 1 && 0 <= pt(1) && pt(1) < srcImage.rows() - 1) {
      double x1 = pt(0) - fipt(0);
      double y1 = pt(1) - fipt(1);
      double x2 = 1 - x1;
      double y2 = 1 - y1;
      val = x2 * y2 * srcImage(fipt(1), fipt(0)) + x1 * y2 * srcImage(fipt(1), fipt(0) + 1) + x2 * y1 * srcImage(
          fipt(1) + 1, fipt(0)) + x1 * y1 * srcImage(fipt(1) + 1, fipt(0) + 1);

    }
    else if (0 <= fipt(0) && fipt(0) < srcImage.cols() && 0 <= fipt(1) && fipt(1) < srcImage.rows()) {
      val = srcImage(fipt(1), fipt(0));
    }
    else if (0 <= cipt(0) && cipt(0) < srcImage.cols() && 0 <= cipt(1) && cipt(1) < srcImage.rows()) {
      val = srcImage(cipt(1), cipt(0));
    }
    else
      val = defaultValue; //templateImage(i / template_cols, i % template_cols); //default to the same as template, so error is 0

    warped(i % template_rows, i / template_rows) = val; //Eigen is Column-major
  }
  return warped;
}

}
