#include "intensity_descriptor.hpp"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <algorithm>
#include <numeric>

#ifdef USE_SSE
#include <emmintrin.h>
#endif

#ifndef ALIGNMENT
#define ALIGNMENT 16
#endif

#include "keypoint.hpp"
#include "internal_utils.hpp"

namespace fovis
{

void
IntensityDescriptorExtractor::initialize() {
  _descriptor_len = _feature_window_size * _feature_window_size;
  // omit a pixel from the descriptor where doing so results in a descriptor
  // length that's a multiple of 16 (e.g., feature window sizes 7, 9, etc.)
  // This makes for faster descriptor comparisons.  If using other feature
  // window sizes (e.g., 8, 10) then there's no significant advantage for
  // omitting that pixel.
  if((_descriptor_len - 1) % 16 == 0) {
    _descriptor_len -= 1;
  }

  // descriptor properties
  _descriptor_stride = round_up_to_multiple(_descriptor_len, ALIGNMENT);
  _brightess_offset_num_sse_ops = _descriptor_stride / 16;
  _num_descriptor_pad_bytes = _descriptor_stride - _descriptor_len;

  // populate the descriptor offset vector.  This vector stores the index
  // offsets of descriptor pixels for a descriptor centered around a given
  // pixel.
  _descriptor_index_offsets = new int[_descriptor_len];
  int i=0;
  int dr = (_feature_window_size - 1) / 2;
  for(int y_offset = -dr; y_offset<=dr; y_offset++) {
    for(int x_offset = -dr; x_offset<=dr && i<_descriptor_len; x_offset++) {
      _descriptor_index_offsets[i] = y_offset * _raw_gray_stride + x_offset;
      i++;
    }
  }
  assert(i == _descriptor_len);

  if(0 != posix_memalign((void**)&_descriptor_brightness_offset, ALIGNMENT, 16))
      fprintf(stderr, "error allocating descriptor brightness offset\n");
}

void
IntensityDescriptorExtractor::populateDescriptorInterp(uint8_t *raw_gray,
                                                       float x, float y,
                                                       uint8_t* descriptor) const
{
  int u = static_cast<int>(x);
  int v = static_cast<int>(y);
  float wright  = (x - u);
  float wbottom = (y - v);
  float wa = (1. - wright) * (1. - wbottom);
  float wb = wright        * (1. - wbottom);
  float wc = (1. - wright) * wbottom;
  float wd = wright        * wbottom;
  int initial_index = v * _raw_gray_stride + u;
  for(int i=0; i<_descriptor_len; i++) {
    int k = initial_index + _descriptor_index_offsets[i];
#ifndef NDEBUG
//    assert(k >= 0 && k < _width*_height);
#endif
    descriptor[i] = (uint8_t)(wa * raw_gray[k] +
        wb * raw_gray[k + 1] +
        wc * raw_gray[k + _raw_gray_stride] +
        wd * raw_gray[k + _raw_gray_stride + 1]);
  }
  normalizeDescriptor(descriptor);

  // zero out some pad bytes of the descriptor so that SIMD
  // operations can be simpler
  memset(descriptor + _descriptor_len, 0, _num_descriptor_pad_bytes);
}

void
IntensityDescriptorExtractor::populateDescriptorAligned(uint8_t *raw_gray,
                                                        int x, int y,
                                                        uint8_t* descriptor) const
{
  // require that the descriptor be 16-byte aligned
  assert(FOVIS_IS_ALIGNED16(descriptor));

  int start = y * _raw_gray_stride + x;
  for (int i=0; i<_descriptor_len; i++) {
    descriptor[i] = raw_gray[start + _descriptor_index_offsets[i]];
  }
  normalizeDescriptor(descriptor);

  // zero out some pad bytes of the descriptor so that SIMD
  // operations can be simpler
  memset(descriptor + _descriptor_len, 0, _num_descriptor_pad_bytes);
}

void
IntensityDescriptorExtractor::normalizeDescriptor(uint8_t* desc) const
{
  assert(FOVIS_IS_ALIGNED16(_descriptor_brightness_offset));

  // get mean of patch
  uint32_t desc_mean = std::accumulate(desc, desc + _descriptor_len, 0)/_descriptor_len;
  // subtract mean, adding offset so 0 -> 128
  if(desc_mean < 128) {
    std::fill(_descriptor_brightness_offset, _descriptor_brightness_offset+16, 128-desc_mean);
    for(int op=0; op<_brightess_offset_num_sse_ops; op++) {
      ((__m128i*)desc)[op] = _mm_adds_epu8(((__m128i*)desc)[op],
        *(__m128i*)_descriptor_brightness_offset);
    }
  } else if (desc_mean > 128){
    std::fill(_descriptor_brightness_offset, _descriptor_brightness_offset+16, desc_mean-128);
    for(int op=0; op<_brightess_offset_num_sse_ops; op++) {
      ((__m128i*)desc)[op] = _mm_subs_epu8(((__m128i*)desc)[op],
        *(__m128i*)_descriptor_brightness_offset);
    }
  }
}

void
IntensityDescriptorExtractor::populateDescriptorsInterp(uint8_t* image,
                                                        const KeypointData* keypoints,
                                                        int num_keypoints,
                                                        uint8_t* descriptors) const
{
  for (int n=0; n<num_keypoints; ++n) {
    const KeypointData& kp_data(keypoints[n]);
    uint8_t* descriptor = descriptors + n*_descriptor_stride;
    populateDescriptorInterp(image, kp_data.kp.u, kp_data.kp.v, descriptor);
    normalizeDescriptor(descriptor);
  }
}

void
IntensityDescriptorExtractor::populateDescriptorsAligned(uint8_t* image,
                                                         const KeypointData* keypoints,
                                                         int num_keypoints,
                                                         uint8_t* descriptors) const
{
  for (int n=0; n<num_keypoints; ++n) {
    const KeypointData& kp_data(keypoints[n]);
    uint8_t* descriptor = descriptors + n*_descriptor_stride;
    populateDescriptorAligned(image,
                              static_cast<int>(kp_data.kp.u),
                              static_cast<int>(kp_data.kp.v),
                              descriptor);
    normalizeDescriptor(descriptor);
  }
}

}
