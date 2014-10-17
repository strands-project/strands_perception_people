#include "normalize_image.hpp"

#include <math.h>
#include "tictoc.hpp"

namespace fovis
{

static void
mean_stddev(const uint8_t * buf,
            int stride, int width, int height,
            float *mean, float *stddev) {
  float sum = 0;
  float sqr_sum = 0;
  for (int i = 0; i < height; ++i) {
    const uint8_t * row_ptr = buf + i*stride;
    for (int j = 0; j < width; ++j) {
      float x = row_ptr[j];
      sum += x;
      sqr_sum += (x * x);
    }
  }
  float n = width * height;
  *mean = sum / n;
  *stddev = sqrt( (sqr_sum - sum * *mean)/n );
}

void
normalize_image(uint8_t * buf, int stride, int width, int height) {
  const float NEW_MEAN = 128.;
  const float NEW_SD = 74.; // approx s.d. of uniform(0, 256) pdf
  float mean, stddev;
  mean_stddev(buf, stride, width, height, &mean, &stddev);
  // TODO not thread safe.
  static uint8_t lookup_table[256];
  for (int i=0; i < 256; ++i) {
    int new_val = (int)((i-mean)*(NEW_SD/stddev) + NEW_MEAN);
    //int new_val = (int)((i-mean) + NEW_MEAN);
    lookup_table[i] = std::min(std::max(new_val, 0), 255);
  }
  for (int i = 0; i < height; ++i) {
    uint8_t* row_ptr = buf + i*stride;
    for (int j = 0; j < width; ++j) {
      row_ptr[j] = lookup_table[row_ptr[j]];
    }
  }
}

}
