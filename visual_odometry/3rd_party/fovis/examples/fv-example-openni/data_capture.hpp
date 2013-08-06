#ifndef __data_capture_hpp__
#define __data_capture_hpp__

#include <XnOS.h>
#include <XnCppWrapper.h>

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
    xn::Context context;
    xn::EnumerationErrors errors;

    xn::DepthGenerator depth_gen;
    xn::DepthMetaData depth_md;
    xn::ImageGenerator image_gen;
    xn::ImageMetaData image_md;

    fovis::DepthImage* depth_image;

    int width;
    int height;

    fovis::CameraIntrinsicsParameters rgb_params;

    float* depth_data;

    uint8_t* gray_buf;
};

}

#endif
