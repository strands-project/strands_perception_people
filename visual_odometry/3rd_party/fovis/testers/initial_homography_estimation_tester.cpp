#include <stdio.h>
#include <iostream>
#include <iomanip>

#include <Eigen/Core>

#include "../libfovis/initial_homography_estimation.hpp"
#include <bot_core/bot_core.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <GL/gl.h>

using namespace std;

#define dump(var) (cerr<<" "#var<<" =[\n"<< setprecision (3)<<var<<"];"<<endl)

static Eigen::ArrayXf flattenMatrix(Eigen::MatrixXf &m)
{
  return Eigen::Map<Eigen::ArrayXf>(m.data(), m.rows() * m.cols());
}

static void warpImage(const uint8_t * image, int width, int height, int rowstride, const Eigen::Matrix3f &H,
    uint8_t * warped_image)
{
  //setup the utility matrices
  Eigen::MatrixXf x = Eigen::VectorXf::LinSpaced(width, 0, width - 1).transpose().replicate(height, 1);
  Eigen::MatrixXf y = Eigen::VectorXf::LinSpaced(height, 0, height - 1).replicate(1, width);
  Eigen::MatrixXf xx = flattenMatrix(x);
  Eigen::MatrixXf yy = flattenMatrix(y);

  Eigen::MatrixXf imageHomogeneousPoints(3, xx.rows());
  imageHomogeneousPoints.row(0) = xx.transpose();
  imageHomogeneousPoints.row(1) = yy.transpose();
  imageHomogeneousPoints.row(2).setOnes();

  Eigen::MatrixXf warpedHomogeneousPoints;
  warpedHomogeneousPoints = H * imageHomogeneousPoints;

  Eigen::MatrixXf warped = Eigen::MatrixXf(height, width);

  const double defaultValue = 128;
  //Bilinear interpolation
  for (int i = 0; i < warpedHomogeneousPoints.cols(); i++) {
    double val;
    Eigen::Vector2f pt = warpedHomogeneousPoints.col(i).head(2) / warpedHomogeneousPoints(2, i);
    Eigen::Vector2i fipt(floor(pt(0)), floor(pt(1)));
    Eigen::Vector2i cipt(ceil(pt(0)), ceil(pt(1)));
    if (0 <= pt(0) && pt(0) < width - 1 && 0 <= pt(1) && pt(1) < height - 1) {
      double x1 = pt(0) - fipt(0);
      double y1 = pt(1) - fipt(1);
      double x2 = 1 - x1;
      double y2 = 1 - y1;
      val = x2 * y2 * image[fipt(1) * rowstride + fipt(0)] + x1 * y2 * image[fipt(1) * rowstride + fipt(0) + 1] + x2
          * y1 * image[(fipt(1) + 1) * rowstride + fipt(0)] + x1 * y1 * image[(fipt(1) + 1) * rowstride + fipt(0) + 1];

    }
    else if (0 <= fipt(0) && fipt(0) < width && 0 <= fipt(1) && fipt(1) < height) {
      val = image[fipt(1) * rowstride + fipt(0)];
    }
    else if (0 <= cipt(0) && cipt(0) < width && 0 <= cipt(1) && cipt(1) < height) {
      val = image[cipt(1) * rowstride + cipt(0)];
    }
    else
      val = defaultValue; //templateImage(i / width, i % width); //default to the same as template, so error is 0

    warped_image[(i % height) * rowstride + (i / height)] = val; //Eigen is Column-major
  }
}

int main(int argc, char** argv)
{

  uint8_t* src_im = (uint8_t*) malloc(640 * 480 * sizeof(uint8_t));

  int width, height, rowstride;
  bot_pgm_read_fname("test_im.pgm", &src_im, &width, &height, &rowstride);

  Eigen::MatrixXf ref_image = Eigen::MatrixXf::Zero(60, 80);
  ref_image.block(15, 25, 30, 30).setConstant(255.0);

  Eigen::Matrix3f H_warp;
  H_warp.setZero();

  double u = 2;
  double v = 1;
  double theta = 1 * M_PI / 180.0;
  int downsample = 0;
  if (argc >= 2)
    u = atof(argv[1]);

  if (argc >= 3)
    v = atof(argv[2]);

  if (argc >= 4)
    theta = atof(argv[3]) * M_PI / 180.0;

  if (argc >= 5)
    downsample = atoi(argv[4]);

  H_warp(0, 0) = cos(theta);
  H_warp(0, 1) = sin(theta);
  H_warp(1, 0) = -sin(theta);
  H_warp(1, 1) = cos(theta);

  H_warp(0, 2) = u;
  H_warp(1, 2) = v;
  H_warp(2, 2) = 1;

  uint8_t* warped_im = (uint8_t*) malloc(width * height * sizeof(uint8_t));
  warpImage(src_im, width, height, rowstride, H_warp, warped_im);

  fovis::InitialHomographyEstimator rotation_estimator;
  rotation_estimator.setTemplateImage(warped_im, width, height, rowstride, downsample);

  rotation_estimator.setTestImage(src_im, width, height, rowstride, downsample);
  double finalRMS = 0;
  Eigen::Matrix3f H_est = rotation_estimator.track(Eigen::Matrix3f::Identity(), 10, &finalRMS);

  double scale_factor = 1 << downsample;
  Eigen::Matrix3f S = Eigen::Matrix3f::Identity() * scale_factor;
  S(2,2)=1;

  //scale homography up to the full size image
  H_est = S * H_est * S.inverse();

  dump(H_warp);
  dump(H_est);
  fprintf(stderr, "finalRMS=%f\n", finalRMS);

  //draw results
  lcm_t * lcm = lcm_create(NULL);
  bot_lcmgl_t* lcmgl = bot_lcmgl_init(lcm, "Init_Homography_Test\n");

  //convert to pixel coords
  bot_lcmgl_push_matrix(lcmgl);
  bot_lcmgl_rotated(lcmgl, -90, 0, 0, 1);
  bot_lcmgl_scalef(lcmgl, 10.0 / width, -10.0 / width, 1);

  // template image
  bot_lcmgl_color3f(lcmgl, 1, 1, 1);
  int template_texid = bot_lcmgl_texture2d(lcmgl, warped_im, width, height, rowstride, BOT_LCMGL_LUMINANCE,
      BOT_LCMGL_COMPRESS_NONE);

  bot_lcmgl_push_matrix(lcmgl);
  bot_lcmgl_translated(lcmgl, 0, height + 10, 0);
  bot_lcmgl_texture_draw_quad(lcmgl, template_texid, 0, 0, 0, 0, height, 0, width, height, 0, width, 0, 0);
  bot_lcmgl_pop_matrix(lcmgl);

  // test image
  bot_lcmgl_color3f(lcmgl, 1, 1, 1);
  int gray_texid = bot_lcmgl_texture2d(lcmgl, src_im, width, height, rowstride, BOT_LCMGL_LUMINANCE,
      BOT_LCMGL_COMPRESS_NONE);
  bot_lcmgl_texture_draw_quad(lcmgl, gray_texid, 0, 0, 0, 0, height, 0, width, height, 0, width, 0, 0);

  //draw the ESM homography estimate
  bot_lcmgl_line_width(lcmgl, 2.0);
  bot_lcmgl_color3f(lcmgl, 1, 1, 0);
  bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
  Eigen::MatrixXf vertices(5, 3);
  vertices << 0, 0, 1, width, 0, 1, width, height, 1, 0, height, 1, 0, 0, 1;

  Eigen::MatrixXf warpedPoints = H_est * vertices.transpose();
  warpedPoints.row(0) = warpedPoints.row(0).array() / warpedPoints.row(2).array();
  warpedPoints.row(1) = warpedPoints.row(1).array() / warpedPoints.row(2).array();
  for (int i = 0; i < warpedPoints.cols(); i++) {
    bot_lcmgl_vertex2f(lcmgl, warpedPoints(0, i), warpedPoints(1, i));
  }
  bot_lcmgl_end(lcmgl);

  bot_lcmgl_pop_matrix(lcmgl);

  bot_lcmgl_switch_buffer(lcmgl);

  return 0;
}
