#include <assert.h>

#include "primesense_depth.hpp"

#include "frame.hpp"
#include "pyramid_level.hpp"

namespace fovis
{

PrimeSenseCalibration::PrimeSenseCalibration(const PrimeSenseCalibrationParameters& params) :
  params(params)
{
  rgb_rectification = new Rectification(params.rgb_params);
}

PrimeSenseCalibration::~PrimeSenseCalibration()
{
  delete rgb_rectification;
}

PrimeSenseDepth::PrimeSenseDepth(const PrimeSenseCalibration* calib)
{
  _calib = calib;
  _ir_width = _calib->getParameters().depth_params.width;
  _ir_height = _calib->getParameters().depth_params.height;

  assert(_ir_width >= 0 && _ir_height >= 0 && _ir_width < 100000 && _ir_height < 100000);

  _disparity_map_max_index = _ir_width * _ir_height - 1;
  _disparity = (uint16_t*) malloc(_ir_width * _ir_height * sizeof(uint16_t) + 2);
  // The entry at the end of the disparity buffer
  // is a placeholder for uninitialized disparity values.
  // It should be greater than any possible disparity so
  // that it will get replaced.
  _disparity[_disparity_map_max_index+1] = 10000;
  _rgb_to_disparity_map = (uint32_t*) malloc(_ir_width * _ir_height * sizeof(uint32_t));
  _uvd1_d_to_xyz_c = new Eigen::Matrix4d(_calib->getDepthUvdToRgbXyz());

}

PrimeSenseDepth::~PrimeSenseDepth()
{
  delete _uvd1_d_to_xyz_c;
  free(_disparity);
  free(_rgb_to_disparity_map);
}

void
PrimeSenseDepth::setDisparityData(const uint16_t* disparity)
{
  // copy disparity image
  std::copy(disparity, disparity+_ir_width * _ir_height, _disparity);

  // initialize the rgb->disparity map
  std::fill(_rgb_to_disparity_map,
            _rgb_to_disparity_map+(_ir_width * _ir_height)+1,
            (_ir_width * _ir_height));

  // == compute RGB pixel coordinates for every pixel in the depth image ==

  Eigen::Matrix<double, 3, 4> depth_to_rgb_uvd = _calib->getDepthUvdToRgbUvw();
  uint16_t min_disparity = 0;
  uint16_t max_disparity = 2046;

  for(int v_disp=0; v_disp<_ir_height; v_disp++) {
    for(int u_disp=0; u_disp<_ir_width; u_disp++) {
      int disparity_index = v_disp * _ir_width + u_disp;
      uint16_t disparity = _disparity[disparity_index];

      if(disparity > max_disparity || disparity < min_disparity)
        continue;

      // compute homogeneous coordinates of matching RGB pixel
      Eigen::Vector4d uvd_depth(u_disp, v_disp, disparity, 1);
      Eigen::Vector3d uvd_rgb = depth_to_rgb_uvd * uvd_depth;

      // normalize to Euclidean coordinates and round off to the nearest pixel.
      if (uvd_rgb[0] < 0 || uvd_rgb[1] < 0 || uvd_rgb[2] <= 0)
        continue ;

      int u_rgb = uvd_rgb[0] / uvd_rgb[2] + 0.5;
      if(u_rgb >= _ir_width)
          continue;

      int v_rgb = uvd_rgb[1] / uvd_rgb[2] + 0.5;
      if(v_rgb >= _ir_height)
          continue;

      int rgb_index = v_rgb * _ir_width + u_rgb;
      uint32_t disparity_index_old = _rgb_to_disparity_map[rgb_index];
      if(disparity < _disparity[disparity_index_old]) {
        _rgb_to_disparity_map[rgb_index] = disparity_index;
      }
    }
  }
}

void
PrimeSenseDepth::getXyz(OdometryFrame * frame)
{
  int num_levels = frame->getNumLevels();
  for(int level_num=0; level_num < num_levels; ++level_num) {
    PyramidLevel* level = frame->getLevel(level_num);

    int num_kp = level->getNumKeypoints();
    for(int kp_ind=0; kp_ind < num_kp; ++kp_ind) {

      KeypointData* kpdata(level->getKeypointData(kp_ind));
      getXyzFast(kpdata);
    }
  }
}

void
PrimeSenseDepth::refineXyz(FeatureMatch * matches,
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

#if 0
bool
PrimeSenseDepth::haveXyz(float u_f, float v_f)
{
  int v = (int)v_f;
  int u = (int)u_f;

  if(u < 0 || v < 0 || u >= _ir_width - 1 || v >= _ir_height - 1)
    return false;

  int ra_index = v * _ir_width + u;
  uint32_t neighbor_disparity_indices[4] = {
    _rgb_to_disparity_map[ra_index],
    _rgb_to_disparity_map[ra_index + 1],
    _rgb_to_disparity_map[ra_index + _ir_width],
    _rgb_to_disparity_map[ra_index + _ir_width + 1]
  };

  // depth data for surrounding pixels?
  for(int i = 0; i<4; i++)
    if(neighbor_disparity_indices[i] < _disparity_map_max_index)
      return true;

  return false;
}
#endif

bool
PrimeSenseDepth::getXyz(int u, int v, Eigen::Vector3d & xyz)
{
  u = (int)(u+0.5);
  v = (int)(v+0.5);

  int rgb_index = v * _ir_width + u;
  uint32_t disparity_index = _rgb_to_disparity_map[rgb_index];
  if(disparity_index > _disparity_map_max_index) {
    return false;
  }

  uint16_t disparity = _disparity[disparity_index];
  if(disparity > 2046) { // TODO account for shadow values?
    return false;
  }

  int u_disp = disparity_index % _ir_width;
  int v_disp = disparity_index / _ir_width;

  // compute feature Cartesian coordinates
  Eigen::Vector4d xyzw = (*_uvd1_d_to_xyz_c) * Eigen::Vector4d(u_disp, v_disp, disparity, 1);
  xyz = xyzw.head<3>() / xyzw.w();

  assert(!isnan(xyz(0)) && !isnan(xyz(1)) && !isnan(xyz(2)));

  return true;
}

bool
PrimeSenseDepth::getXyzFast(KeypointData* kpdata)
{
  int u = (int)(kpdata->rect_base_uv(0)+0.5);
  int v = (int)(kpdata->rect_base_uv(1)+0.5);

  int rgb_index = v * _ir_width + u;
  uint32_t disparity_index = _rgb_to_disparity_map[rgb_index];
  if(disparity_index > _disparity_map_max_index) {
    kpdata->has_depth = false;
    kpdata->xyzw = Eigen::Vector4d(NAN, NAN, NAN, NAN);
    kpdata->xyz = Eigen::Vector3d(NAN, NAN, NAN);
    kpdata->disparity = NAN;
    return false;
  }

  uint16_t disparity = _disparity[disparity_index];
  if(disparity > 2046) { // TODO account for shadow values?
    kpdata->has_depth = false;
    kpdata->xyzw = Eigen::Vector4d(NAN, NAN, NAN, NAN);
    kpdata->xyz = Eigen::Vector3d(NAN, NAN, NAN);
    kpdata->disparity = NAN;
    return false;
  }

  int u_disp = disparity_index % _ir_width;
  int v_disp = disparity_index / _ir_width;

  // compute feature Cartesian coordinates
  Eigen::Vector4d uvd_depth(u_disp, v_disp, disparity, 1);
  kpdata->xyzw = (*_uvd1_d_to_xyz_c) * uvd_depth;
  kpdata->xyz = kpdata->xyzw.head<3>() / kpdata->xyzw.w();
  kpdata->has_depth = true;
  kpdata->disparity = disparity;

  assert(!isnan(kpdata->xyz(0)) && !isnan(kpdata->xyz(1)) && !isnan(kpdata->xyz(2)));

  return true;
}

bool
PrimeSenseDepth::getXyzInterp(KeypointData* kpdata)
{
  float u_f = kpdata->rect_base_uv(0);
  float v_f = kpdata->rect_base_uv(1);
  int v = (int)v_f;
  int u = (int)u_f;
  float wright  = (u_f - u);
  float wbottom = (v_f - v);

  // can't handle borders
  assert(u >= 0 && v >= 0 && u < _ir_width - 1 && v < _ir_height - 1);

  float w[4] = {
    (1 - wright) * (1 - wbottom),
    wright * (1 - wbottom),
    (1 - wright) * wbottom,
    wright * wbottom
  };

  int ra_index = v * _ir_width + u;
  uint32_t neighbor_disparity_indices[4] = {
    _rgb_to_disparity_map[ra_index],
    _rgb_to_disparity_map[ra_index + 1],
    _rgb_to_disparity_map[ra_index + _ir_width],
    _rgb_to_disparity_map[ra_index + _ir_width + 1]
  };

  // missing any depth data for surrounding pixels?
  int num_missing_data = 0;
  for(int i = 0; i<4; i++)
    if(neighbor_disparity_indices[i] >= _disparity_map_max_index)
      num_missing_data++;

  if(num_missing_data == 4) {
    // missing all surrounding depth data.
    kpdata->xyzw = Eigen::Vector4d(NAN, NAN, NAN, NAN);
    kpdata->xyz = Eigen::Vector3d(NAN, NAN, NAN);
    kpdata->disparity = NAN;
    return false;
  }

  // if any of the surrounding depth data is missing, just clamp to the
  // nearest pixel.  interpolation gets messy if we try to do otherwise
  if(num_missing_data) {
    float wmax = -1;
    int wmax_disparity_ind = -1;
    for(int i=0; i<4; i++) {
      if(neighbor_disparity_indices[i] < _disparity_map_max_index &&
          w[i] > wmax) {
        wmax_disparity_ind = i;
        wmax = w[i];
      }
    }
    int v = wmax_disparity_ind / _ir_width;
    int u = wmax_disparity_ind % _ir_width;
    uint16_t disparity = _disparity[wmax_disparity_ind];
    Eigen::Vector4d uvd1_d(u, v, disparity, 1);
    kpdata->xyzw = (*_uvd1_d_to_xyz_c) * uvd1_d;
    kpdata->disparity = disparity;
  } else {
    Eigen::Vector4d uvd1(0, 0, 0, 1);
    for(int i=0; i<4; i++) {
      int nu = neighbor_disparity_indices[i] % _ir_width;
      int nv = neighbor_disparity_indices[i] / _ir_width;
      int ndisparity = _disparity[neighbor_disparity_indices[i]];
      uvd1(0) += w[i] * nu;
      uvd1(1) += w[i] * nv;
      uvd1(2) += w[i] * ndisparity;
    }
    kpdata->xyzw = (*_uvd1_d_to_xyz_c) * uvd1;
    kpdata->disparity = uvd1(2);
  }
  kpdata->xyz = kpdata->xyzw.head<3>() / kpdata->xyzw.w();
  return true;
}

}
