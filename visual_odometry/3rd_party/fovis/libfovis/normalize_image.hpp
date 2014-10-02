#ifndef __fovis_normalize_image_hpp__
#define __fovis_normalize_image_hpp__

#include <inttypes.h>

namespace fovis
{

/**
 * \brief Normalize image intensities in place to approximately have mean 128 and sd 74.
 */
void normalize_image(uint8_t * buf, int stride, int width, int height);

}

#endif
