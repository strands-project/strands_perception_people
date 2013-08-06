#include "rectification.hpp"


#include <inttypes.h>

#include <iostream>

namespace fovis
{

Rectification::Rectification(const CameraIntrinsicsParameters& params) :
  _map_x(NULL),
  _map_y(NULL)
{
  _input_camera = params;
  _rotation = new Eigen::Matrix3d(Eigen::Matrix3d::Identity());
  _rectified_camera = params;
  _rectified_camera.k1 = 0;
  _rectified_camera.k2 = 0;
  _rectified_camera.k3 = 0;
  _rectified_camera.p1 = 0;
  _rectified_camera.p2 = 0;
  populateMap();
}

Rectification::Rectification(const CameraIntrinsicsParameters& input_camera_params,
        const Eigen::Matrix3d& rotation,
        const CameraIntrinsicsParameters& rectified_camera_params) :
  _map_x(NULL),
  _map_y(NULL)
{
  _input_camera = input_camera_params;
  _rotation = new Eigen::Matrix3d(rotation);
  _rectified_camera = rectified_camera_params;
  populateMap();
}

void
Rectification::populateMap()
{
  assert(_input_camera.width > 0 && _input_camera.height > 0);
  delete[] _map_x;
  delete[] _map_y;
  _map_x = new float[_input_camera.width * _input_camera.height];
  _map_y = new float[_input_camera.width * _input_camera.height];

  int input_width = _input_camera.width;
  int input_height = _input_camera.height;
  double fx  = _input_camera.fx;
  double fy  = _input_camera.fy;
  double cx  = _input_camera.cx;
  double cy  = _input_camera.cy;
  double fxp = _rectified_camera.fx;
  double fyp = _rectified_camera.fy;
  double cxp = _rectified_camera.cx;
  double cyp = _rectified_camera.cy;
  double k1  = _input_camera.k1;
  double k2  = _input_camera.k2;
  double k3  = _input_camera.k3;
  double p1  = _input_camera.p1;
  double p2  = _input_camera.p2;

  // this code is based on OpenCV cvUndistortPoints
  for (int y = 0; y < input_height; ++y) {
    for (int x = 0; x < input_width; ++x) {
      // normalize according to principal point and focal length
      double x1 = (x - cx)/fx;
      double y1 = (y - cy)/fy;
      double x0 = x1;
      double y0 = y1;
      // Iteratively undistort point
      for(int j = 0; j < 5; ++j) {
        double r2 = x1*x1 + y1*y1;
        double icdist = 1./(1. + ((k3*r2 + k2)*r2 + k1)*r2);
        double deltaX = 2*p1*x1*y1 + p2*(r2 + 2*x1*x1);
        double deltaY = p1*(r2 + 2*y1*y1) + 2*p2*x1*y1;
        x1 = (x0 - deltaX)*icdist;
        y1 = (y0 - deltaY)*icdist;
      }
      // rectification
      Eigen::Vector3d xyw = (*_rotation) * Eigen::Vector3d(x1,  y1, 1);
      x1 = xyw(0) / xyw(2);
      y1 = xyw(1) / xyw(2);
      // projection for rectified rectified image
      x1 = x1 * fxp + cxp;
      y1 = y1 * fyp + cyp;
      _map_x[y * input_width + x] = x1;
      _map_y[y * input_width + x] = y1;
    }
  }
}

Rectification::~Rectification()
{
  delete[] _map_x;
  delete[] _map_y;
  delete _rotation;
  _map_x = NULL;
  _map_y = NULL;
}

Rectification*
Rectification::makeCopy() const
{
  Rectification* result = new Rectification();
  result->_input_camera = _input_camera;
  result->_rotation = new Eigen::Matrix3d(*_rotation);
  result->_rectified_camera = _rectified_camera;
  int num_elem = _input_camera.width * _input_camera.height;
  result->_map_x = new float[num_elem];
  result->_map_y = new float[num_elem];
  std::copy(_map_x, _map_x+num_elem, result->_map_x);
  std::copy(_map_y, _map_y+num_elem, result->_map_y);
  return result;
}

#if 0
// just a quick function for debugging, not for serious use
void fwd_map(int width, int height, const float * mapx, const float * mapy, uint8_t *img,
             uint8_t *img_out);
void
fwd_map(int width, int height, const float * mapx, const float * mapy,
        uint8_t *img, uint8_t *img_out) {
  for (int i=0; i < height; ++i) {
    for (int j=0; j < width; ++j) {
      int new_x = (int)mapx[width*i + j];
      int new_y = (int)mapy[width*i + j];
      if (new_x < 0 || new_x >= width) continue;
      if (new_y < 0 || new_y >= height) continue;
      img_out[width*new_y + new_x] = img[width*i + j];
    }
  }
}
#endif

} /*  */
