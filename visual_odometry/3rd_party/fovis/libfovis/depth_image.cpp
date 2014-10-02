#include <math.h>
#include <assert.h>
#include "depth_image.hpp"
#include "frame.hpp"
#include "feature_match.hpp"

namespace fovis
{

DepthImage::DepthImage(const CameraIntrinsicsParameters& rgb_cam_params,
    int depth_width, int depth_height)
{
  _rgb_width = rgb_cam_params.width;
  _rgb_height = rgb_cam_params.height;

  _depth_width = depth_width;
  _depth_height = depth_height;
  _x_scale = (float)_depth_width / (float)_rgb_width;
  _y_scale = (float)_depth_height / (float)_rgb_height;

  int num_depth_pixels = _depth_width * _depth_height;
  _depth_data = new float[num_depth_pixels];

  int num_rgb_pixels = _rgb_width * _rgb_height;
  _rays = new Eigen::Matrix<double, 3, Eigen::Dynamic>(3, num_rgb_pixels);

  _fx_inv = 1.0 / rgb_cam_params.fx;
  _fy_inv = 1.0 / rgb_cam_params.fy;
  _neg_cx_div_fx = - rgb_cam_params.cx * _fx_inv;
  _neg_cy_div_fy = - rgb_cam_params.cy * _fy_inv;

  // precompute RGB rays
  int rindex = 0;
  for(int v=0; v<_rgb_height; v++) {
    for(int u=0; u<_rgb_width; u++) {
      (*_rays)(0, rindex) = u * _fx_inv + _neg_cx_div_fx;
      (*_rays)(1, rindex) = v * _fy_inv + _neg_cy_div_fy;
      (*_rays)(2, rindex) = 1;
      rindex++;
    }
  }
}

DepthImage::~DepthImage()
{
  delete[] _depth_data;
  delete _rays;
}

void 
DepthImage::setDepthImage(const float* depth_image)
{
  // copy depth image
  int num_depth_pixels = _depth_width * _depth_height;
  memcpy(_depth_data, depth_image, num_depth_pixels * sizeof(float));
}

bool
DepthImage::haveXyz(int u, int v)
{
  return !isnan(_depth_data[rgbToDepthIndex(u, v)]);
}

void
DepthImage::getXyz(OdometryFrame * frame) 
{
  int num_levels = frame->getNumLevels();
  for(int level_num=0; level_num < num_levels; ++level_num) {
    PyramidLevel* level = frame->getLevel(level_num);

    int num_kp = level->getNumKeypoints();
    for(int kp_ind=0; kp_ind < num_kp; ++kp_ind) {

      KeypointData* kpdata(level->getKeypointData(kp_ind));

      int u = (int)(kpdata->rect_base_uv(0)+0.5);
      int v = (int)(kpdata->rect_base_uv(1)+0.5);

      kpdata->disparity = NAN;

      float z = _depth_data[rgbToDepthIndex(u, v)];
      if(isnan(z)) {
        kpdata->has_depth = false;
        kpdata->xyzw = Eigen::Vector4d(NAN, NAN, NAN, NAN);
        kpdata->xyz = Eigen::Vector3d(NAN, NAN, NAN);
      } else {
        kpdata->has_depth = true;
        kpdata->xyz = z * _rays->col(v * _rgb_width + u);
        kpdata->xyzw.head<3>() = kpdata->xyz;
        kpdata->xyzw.w() = 1;
      }
    }
  }
}

void
DepthImage::refineXyz(FeatureMatch * matches,
                           int num_matches,
                           OdometryFrame * frame)
{
  for (int m_ind = 0; m_ind < num_matches; m_ind++) {
    FeatureMatch& match = matches[m_ind];
    if (match.status == MATCH_NEEDS_DEPTH_REFINEMENT) {
      if (getXyzInterp(&match.refined_target_keypoint)) {
        match.status = MATCH_OK;
      } else {
        match.status = MATCH_REFINEMENT_FAILED;
        match.inlier = false;
      }
    }
  }
}


bool
DepthImage::getXyzInterp(KeypointData* kpdata)
{
  float u_f = kpdata->rect_base_uv(0);
  float v_f = kpdata->rect_base_uv(1);
  float v_f_d = v_f * _y_scale;
  float u_f_d = u_f * _x_scale;
  int v = (int)v_f_d;
  int u = (int)u_f_d;
  float wright  = (u_f_d - u);
  float wbottom = (v_f_d - v);

  // can't handle borders
  assert(u >= 0 && v >= 0 && u < _depth_width - 1 && v < _depth_height - 1);

  float w[4] = { 
    (1 - wright) * (1 - wbottom),
    wright * (1 - wbottom),
    (1 - wright) * wbottom,
    wright * wbottom
  };

  int depth_index = v * _depth_width + u;
  double depths[4] = {
    _depth_data[depth_index + 0],
    _depth_data[depth_index + 1],
    _depth_data[depth_index + _depth_width],
    _depth_data[depth_index + _depth_width + 1]
  };

  // missing any depth data for surrounding pixels?
  int num_missing_data = 0;
  for(int i = 0; i<4; i++)
    if(isnan(depths[i]))
      num_missing_data++;

  if(num_missing_data == 4) {
    // missing all surrounding depth data.
    return false;
  }

  if(num_missing_data) {
    // if any of the surrounding depth data is missing, just clamp to the
    // nearest pixel.  interpolation gets messy if we try to do otherwise
    float wmax = -1;
    double z = NAN;
    for(int i=0; i<4; i++) {
      if(isnan(depths[i]) && w[i] > wmax) {
        z = depths[i];
        wmax = w[i];
      }
    }
    kpdata->xyz.x() = z * (u_f * _fx_inv + _neg_cx_div_fx);
    kpdata->xyz.y() = z * (v_f * _fy_inv + _neg_cy_div_fy);
    kpdata->xyz.z() = z;
  } else {
    double z = 0;
    for(int i=0; i<4; i++)
      z += depths[i] * w[i];
    kpdata->xyz.x() = z * (u_f * _fx_inv + _neg_cx_div_fx);
    kpdata->xyz.y() = z * (v_f * _fy_inv + _neg_cy_div_fy);
    kpdata->xyz.z() = z;
  }
  kpdata->xyzw.head<3>() = kpdata->xyz;
  kpdata->xyzw.w() = 1;
  return true;
}

}
