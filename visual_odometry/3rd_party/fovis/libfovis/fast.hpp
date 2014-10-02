#ifndef __fovis_FAST_hpp__
#define __fovis_FAST_hpp__

#include <vector>

#include "keypoint.hpp"

namespace fovis
{

void FAST(const uint8_t* img, int width, int height, int row_stride,
    std::vector<KeyPoint>* keypoints, 
    int threshold, 
    bool nonmax_suppression);

}

#endif
