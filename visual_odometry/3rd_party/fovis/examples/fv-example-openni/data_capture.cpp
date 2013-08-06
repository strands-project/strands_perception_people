#include <stdio.h>

#include "data_capture.hpp"

#define CHECK_STATUS(rc, msg) if((rc) != XN_STATUS_OK) { \
  fprintf(stderr, "%s: %s\n", (msg), xnGetStatusString(rc)); return false; }

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
  // Unfortunately, the OpenNI API doesn't seem to expose them.
  rgb_params.fx = 528.49404721; 
  rgb_params.fy = rgb_params.fx;
  rgb_params.cx = width / 2.0;
  rgb_params.cy = height / 2.0;

  depth_image = new fovis::DepthImage(rgb_params, width, height);
  depth_data = new float[width * height];
  gray_buf = new uint8_t[width * height];
}

DataCapture::~DataCapture()
{
  delete[] depth_data;
  delete[] gray_buf;
}

bool
DataCapture::initialize()
{
  XnStatus rc = context.Init();
  CHECK_STATUS(rc, "Initializing device context");

  printf("Initializing image stream\n");
  image_gen.Create(context);
  rc = image_gen.Create(context);
  CHECK_STATUS(rc, "Initializing image stream");

  // set output format to RGB
  image_gen.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);

  XnMapOutputMode image_mode;
  image_mode.nXRes = width;
  image_mode.nYRes = height;
  image_mode.nFPS = 30;
  image_gen.SetMapOutputMode(image_mode);
  CHECK_STATUS(rc, "Setting image output mode");


  printf("Initializing depth stream\n");
  rc = depth_gen.Create(context);
  CHECK_STATUS(rc, "Initializing depth stream");

  depth_gen.SetMapOutputMode(image_mode);
  CHECK_STATUS(rc, "Setting depth output mode");

  depth_gen.GetMetaData(depth_md);
  printf("Depth offset: %d %d\n", depth_md.XOffset(), depth_md.YOffset());
  // XXX do we need to do something with the depth offset?

  // set the depth image viewpoint
  depth_gen.GetAlternativeViewPointCap().SetViewPoint(image_gen);

  // read off the depth camera field of view.  This is the FOV corresponding to
  // the IR camera viewpoint, regardless of the alternative viewpoint settings.
  XnFieldOfView fov;
  rc = depth_gen.GetFieldOfView(fov);
  return true;
}

bool
DataCapture::startDataCapture()
{
  // start data capture
  printf("Starting data capture\n");
  XnStatus rc = context.StartGeneratingAll();
  CHECK_STATUS(rc, "Starting data capture");
  return true;
}

bool
DataCapture::stopDataCapture()
{
  context.StopGeneratingAll();
  context.Shutdown();
  return true;
}

bool
DataCapture::captureOne()
{
  // Read a new frame
  XnStatus rc = context.WaitAndUpdateAll();
  CHECK_STATUS(rc, "Reading frame");

  // grab the image data
  image_gen.GetMetaData(image_md);
  const XnRGB24Pixel* rgb_data = image_md.RGB24Data();

  // convert to grayscale.
  int num_rgb_pixels = width * height;
  for(int i=0; i<num_rgb_pixels; i++) {
//    gray_buf[i] = (int)round((rgb_data->nRed + 
//                              rgb_data->nGreen + 
//                              rgb_data->nBlue) / 3.0);
    gray_buf[i] = (int)round(0.2125 * rgb_data->nRed + 
                             0.7154 * rgb_data->nGreen + 
                             0.0721 * rgb_data->nBlue);
    rgb_data++;
  }

  // grab the depth data
  depth_gen.GetMetaData(depth_md);
  int depth_data_nbytes = width * height * sizeof(uint16_t);
  const uint16_t* depth_data_u16 = depth_md.Data();

  // convert to meters, and set unknown depth values to NAN
  int num_depth_pixels = width * height;
  for(int i=0; i<num_depth_pixels; i++) {
    uint16_t d = depth_data_u16[i];
    if(d != 0) {
      depth_data[i] = d * 1e-3;
    } else {
      depth_data[i] = NAN;
    }
  }

  depth_image->setDepthImage(depth_data);
  return true;
}

}
