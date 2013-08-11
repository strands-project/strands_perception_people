#include "gauss_pyramid.h"

/**
 * Applies a 1x5 horizontal gaussian filter to every other column in the 
 * source image, and transposes the result.  Borders are reflected.
 * Filter kernel is 1/16 * [ 1 4 6 4 1 ], which corresponds approximately to
 * \sigma=1.0 
 */
static void
filter_horiz_transpose_8u_C1R(const uint8_t* src, int src_stride, int src_width,
        int src_height, uint8_t* dest, int dst_stride)
{
    int dst_max_row = src_width/2 - 1;
    int dst_col;

    for(dst_col=0; dst_col<src_height; dst_col++) {
        const uint8_t* s = &src[dst_col * src_stride];
        uint16_t sum = 0;
        int dst_row;

        // left border
        sum = s[0] + 4 * s[1] + 3 * s[2];
        dest[dst_col] = sum >> 3;

        // middle
        for(dst_row=1; dst_row<dst_max_row; dst_row++) {
            sum = s[0] + 4 * s[1] + 6 * s[2] + 4 * s[3] + s[4];
            dest[dst_row * dst_stride + dst_col] = sum / 16;
            s+=2;
        }

        // right border
        if(src_width & 0x1) {
            sum = 3 * s[0] + 4 * s[1] + s[2];
            dest[dst_max_row * dst_stride + dst_col] = sum >> 3;
        } else {
            sum = s[0] + 4 * s[1] + 7 * s[2] + 4 * s[3];
            dest[dst_max_row * dst_stride + dst_col] = sum >> 4;
        }
    }
}

int 
gauss_pyr_down_get_buf_size_8u_C1R(int width, int height)
{
    return width * height / 2;
}

int
gauss_pyr_down_8u_C1R(const uint8_t* src, int src_stride, int width,
        int height, uint8_t* dest, int dst_stride, uint8_t* buf)
{
    filter_horiz_transpose_8u_C1R(src, src_stride, width, height, buf, height);
    filter_horiz_transpose_8u_C1R(buf, height, height, width/2, dest, dst_stride);
    return 0;
}
