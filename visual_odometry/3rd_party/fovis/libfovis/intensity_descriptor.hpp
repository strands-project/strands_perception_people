#ifndef __fovis_gray_descriptor_hpp__
#define __fovis_gray_descriptor_hpp__

#include <inttypes.h>
#include <stdlib.h>

namespace fovis
{

class KeypointData;

/**
 * \ingroup FovisCore
 * \brief Extracts mean-normalized intensity patch around a pixel.
 *
 */
class IntensityDescriptorExtractor {
 public:
   /**
    * \param raw_gray_stride specifies the number of bytes separating each row
    * of the input image.
    * \param feature_window_size the size of the descriptor window.
    */
  IntensityDescriptorExtractor(int raw_gray_stride, int feature_window_size)
  : _raw_gray_stride(raw_gray_stride),
    _feature_window_size(feature_window_size) {
    initialize();
  }

  virtual ~IntensityDescriptorExtractor () {
    delete[] _descriptor_index_offsets;
    free(_descriptor_brightness_offset);
  }

  /**
   * Computes a single descriptor using bilinear interpolation at every point
   * on the descriptor.  This performs 4 floating point multiplications for
   * every pixel in the descriptor.
   *
   * Assumes that the output parameter \p is 16-byte aligned.
   */
  void populateDescriptorInterp(uint8_t *image,
                                float x, float y,
                                uint8_t* descriptor) const;

  /**
   * Computes a single descriptor.  Assumes that the output parameter \p
   * descriptor is 16-byte aligned.
   */
  void populateDescriptorAligned(uint8_t *image,
                                 int x, int y,
                                 uint8_t* descriptor) const;

  /**
   * Compute many descriptors using bilinear interpolation.
   *
   * \param image input image used for descriptor computation.
   * \param keypoints keypoints for descriptor computation.
   * \param num_keypoints the number of keypoints.
   * \param descriptors Must be 16-byte aligned and pre-allocated to
   * \p num_keypoints * 16 * getDescriptorStride() bytes.
   */
  void populateDescriptorsInterp(uint8_t* image,
                                 const KeypointData* keypoints,
                                 int num_keypoints,
                                 uint8_t* descriptors) const;

  /**
   * Compute many descriptors.
   *
   * \param image input image used for descriptor computation.
   * \param keypoints keypoints for descriptor computation.
   * \param num_keypoints the number of keypoints.
   * \param descriptors Must be 16-byte aligned and pre-allocated to
   * \p num_keypoints * 16 * getDescriptorStride() bytes.
   */
  void populateDescriptorsAligned(uint8_t* image,
                                  const KeypointData* keypoints,
                                  int num_keypoints,
                                  uint8_t* descriptors) const;

  /**
   * \return the number of bytes that should separate descriptors in a vector
   * of keypoint descriptors.  This is usually a multiple of 16 so that each
   * descriptor is 16-byte aligned.
   */
  int getDescriptorStride() const {
    return _descriptor_stride;
  }

  /**
   * \return an array of offset indices indicating the image offset of each
   * descriptor pixel from the descriptor center pixel.
   */
  const int* getDescriptorIndexOffsets() const {
    return _descriptor_index_offsets;
  }

  /**
   * \return the number of useful bytes in each descriptor.
   */
  int getDescriptorLength() const {
    return _descriptor_len;
  }

 private:
  IntensityDescriptorExtractor(const IntensityDescriptorExtractor& other);
  IntensityDescriptorExtractor& operator=(const IntensityDescriptorExtractor& other);

  void initialize();
  void normalizeDescriptor(uint8_t* desc) const;

  int _raw_gray_stride;

  int _num_descriptor_pad_bytes;
  uint8_t* _descriptor_brightness_offset;
  int _brightess_offset_num_sse_ops;

  int _descriptor_len;
  int _feature_window_size;
  int _descriptor_stride;
  int* _descriptor_index_offsets;

};

} /*  */
#endif
