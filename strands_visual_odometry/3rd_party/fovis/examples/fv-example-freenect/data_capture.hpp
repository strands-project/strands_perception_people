#ifndef __data_capture_freenect_hpp__
#define __data_capture_freenect_hpp__

#include <libfreenect.h>

#include <fovis/fovis.hpp>

namespace fovis_example
{

class DataCapture
{
  public:
    DataCapture();
    ~DataCapture();

    bool initialize();

    bool startDataCapture();

    bool stopDataCapture();

    bool captureOne();

    fovis::DepthImage* getDepthImage() {
      return depth_image;
    }

    const fovis::CameraIntrinsicsParameters& getRgbParameters() const {
      return rgb_params;
    }

    const uint8_t* getGrayImage() {
      return gray_buf;
    }

  private:
    static void depth_cb(freenect_device *dev, void *data, uint32_t timestamp);
    static void image_cb(freenect_device *dev, void *data, uint32_t timestamp);

    void DepthCallback(void* data, uint32_t timestamp);
    void ImageCallback(void* data, uint32_t timestamp);

    freenect_context *f_ctx;
    freenect_device *f_dev;

    int freenect_angle;
    int device_number;

    fovis::DepthImage* depth_image;

    int width;
    int height;

    fovis::CameraIntrinsicsParameters rgb_params;

    bool have_image;
    bool have_depth;

    float* depth_data;

    uint8_t* gray_buf;
};

}

#endif
