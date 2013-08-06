#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <iostream>
#include <string>

#include <fovis/fovis.hpp>

#include "data_capture.hpp"

#define dbg(...) fprintf(stderr, __VA_ARGS__)

sig_atomic_t shutdown_flag = 0;
static void
sig_action(int signal, siginfo_t *s, void *user)
{
  shutdown_flag = 1;
}

std::string
isometryToString(const Eigen::Isometry3d& m)
{
  char result[80];
  memset(result, 0, sizeof(result));
  Eigen::Vector3d xyz = m.translation();
  Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
  snprintf(result, 79, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f",
      xyz(0), xyz(1), xyz(2),
      rpy(0) * 180/M_PI, rpy(1) * 180/M_PI, rpy(2) * 180/M_PI);
  return std::string(result);
}

int main(int argc, char **argv)
{
  // initialize the device
  fovis_example::DataCapture* cap = new fovis_example::DataCapture();
  if(!cap->initialize()) {
    fprintf(stderr, "Unable to initialize Kinect sensor\n");
    return 1;
  }
  if(!cap->startDataCapture()) {
    fprintf(stderr, "Unable to start data capture\n");
    return 1;
  }

  // get the RGB camera parameters of our device
  fovis::Rectification rect(cap->getRgbParameters());

  fovis::VisualOdometryOptions options =
      fovis::VisualOdometry::getDefaultOptions();
  // If we wanted to play around with the different VO parameters, we could set
  // them here in the "options" variable.

  // setup the visual odometry
  fovis::VisualOdometry* odom = new fovis::VisualOdometry(&rect, options);

  // exit cleanly on CTL-C
  struct sigaction new_action;
  new_action.sa_sigaction = sig_action;
  sigemptyset(&new_action.sa_mask);
  new_action.sa_flags = 0;
  sigaction(SIGINT, &new_action, NULL);
  sigaction(SIGTERM, &new_action, NULL);
  sigaction(SIGHUP, &new_action, NULL);

  while(!shutdown_flag) {
    if(!cap->captureOne()) {
      fprintf(stderr, "Capture failed\n");
      break;
    }

    odom->processFrame(cap->getGrayImage(), cap->getDepthImage());

    // get the integrated pose estimate.
    Eigen::Isometry3d cam_to_local = odom->getPose();

    // get the motion estimate for this frame to the previous frame.
    Eigen::Isometry3d motion_estimate = odom->getMotionEstimate();

    // display the motion estimate.  These values are all given in the RGB
    // camera frame, where +Z is forward, +X points right, +Y points down, and
    // the origin is located at the focal point of the RGB camera.
    std::cout << isometryToString(cam_to_local) << " " <<
      isometryToString(motion_estimate) << "\n";
  }

  printf("Shutting down\n");
  cap->stopDataCapture();
  delete odom;
  delete cap;
  return 0;
}
