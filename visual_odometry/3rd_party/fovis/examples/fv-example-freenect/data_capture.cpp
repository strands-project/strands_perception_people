#include <stdio.h>

#include "data_capture.hpp"

namespace fovis_example
{

DataCapture::DataCapture()
{
  width = 640;
  height = 480;

  memset(&rgb_params, 0, sizeof(fovis::CameraIntrinsicsParameters));
  rgb_params.width = width;
  rgb_params.height = height;

  // TODO read these values from the camera somehow, instead of hard-coding it
  rgb_params.fx = 528.49404721;
  rgb_params.fy = rgb_params.fx;
  rgb_params.cx = width / 2.0;
  rgb_params.cy = height / 2.0;

  depth_image = new fovis::DepthImage(rgb_params, width, height);
  depth_data = new float[width * height];
  gray_buf = new uint8_t[width * height];

  freenect_angle = 0;
  device_number = 0;
}

DataCapture::~DataCapture()
{
  delete[] depth_data;
  delete[] gray_buf;
}

bool
DataCapture::initialize()
{
  // initialize the kinect device
  if (freenect_init(&f_ctx, NULL) < 0) {
    printf("freenect_init() failed\n");
    return false;
  }

  freenect_set_log_level(f_ctx, FREENECT_LOG_ERROR);

  int num_devices = freenect_num_devices(f_ctx);
  printf("Number of devices found: %d\n", num_devices);

  if (num_devices < 1)
    return false;

  if (freenect_open_device(f_ctx, &f_dev, device_number) < 0) {
    printf("Could not open device\n");
    return false;
  }

  freenect_set_user(f_dev, this);

  freenect_frame_mode vmode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);
  freenect_frame_mode dmode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED);

  freenect_set_video_mode(f_dev, vmode);
  freenect_set_depth_mode(f_dev, dmode);

  return true;
}

bool
DataCapture::startDataCapture()
{
  // start data capture
  printf("Starting data capture\n");

  freenect_set_tilt_degs(f_dev, freenect_angle);
  freenect_set_led(f_dev, LED_OFF);
  freenect_set_depth_callback(f_dev, &DataCapture::depth_cb);
  freenect_set_video_callback(f_dev, &DataCapture::image_cb);

  freenect_start_depth(f_dev);
  freenect_start_video(f_dev);

  return true;
}

bool
DataCapture::stopDataCapture()
{
  freenect_stop_depth(f_dev);
  freenect_stop_video(f_dev);

  freenect_close_device(f_dev);
  freenect_shutdown(f_ctx);

  return true;
}

bool
DataCapture::captureOne()
{
  while (freenect_process_events(f_ctx) >= 0) {
    if (have_image && have_depth) {
      have_image = false;
      have_depth = false;
      return true;
    }
  }
  return false;
}

void
DataCapture::depth_cb(freenect_device *dev, void *data, uint32_t timestamp)
{
  DataCapture* self = (DataCapture*)(freenect_get_user(dev));
  self->DepthCallback(data, timestamp);
}

void
DataCapture::image_cb(freenect_device *dev, void *data, uint32_t timestamp)
{
  DataCapture* self = (DataCapture*)(freenect_get_user(dev));
  self->ImageCallback(data, timestamp);
}

void
DataCapture::DepthCallback(void* data, uint32_t timestamp)
{
  have_depth = true;

  uint16_t* depth_mm = (uint16_t*)data;
  int num_pixels = width * height;
  for(int i=0; i<num_pixels; i++) {
    uint16_t d = depth_mm[i];
    if(d != 0) {
      depth_data[i] = d * 1e-3;
    } else {
      depth_data[i] = NAN;
    }
  }
  depth_image->setDepthImage(depth_data);
}

void
DataCapture::ImageCallback(void* data, uint32_t timestamp)
{
  have_image = true;

  int num_pixels = width * height;
  uint8_t* rgb_pixel = (uint8_t*)data;
  uint8_t* gray_pixel = gray_buf;
  for(int i=0; i<num_pixels; i++) {
    gray_pixel[0] = (rgb_pixel[0] + rgb_pixel[1] + rgb_pixel[2]) / 3;
    gray_pixel++;
    rgb_pixel += 3;
  }
}

}
