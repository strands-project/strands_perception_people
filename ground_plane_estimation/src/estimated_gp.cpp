// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>

#include <string.h>
#include <boost/thread.hpp>

#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>

#include "Matrix.h"
#include "Vector.h"
#include "Camera.h"
#include "pointcloud.h"
#include "Globals.h"
#include "groundplaneestimator.h"


#include "ground_plane_estimation/GroundPlane.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub_ground_plane;

cv::Mat img_depth_;
cv_bridge::CvImagePtr cv_depth_ptr;	// cv_bridge for depth image

GroundPlaneEstimator GPEstimator;

bool checkParam(bool success, std::string param) {
    if(!success) {
        ROS_FATAL("Parameter: '%s' could not be found! Please make sure that the parameters are available on the parameter server or start with 'load_params_from_file:=true'", param.c_str());
    }
    return success;
}

bool ReadConfigParams(ros::NodeHandle n)
{
    bool success = true;

    std::string ns = ros::this_node::getName();
    ns += "/";

    success = checkParam(n.getParam(ns+"nrInter_ransac", Globals::nrInter_ransac), ns+"nrInter_ransac") && success;
    success = checkParam(n.getParam(ns+"numberOfPoints_reconAsObstacle", Globals::numberOfPoints_reconAsObstacle), ns+"numberOfPoints_reconAsObstacle") && success;

    return success;
}


void callback(const ImageConstPtr &depth,  const CameraInfoConstPtr &info)
{

    // Get depth
    cv_depth_ptr = cv_bridge::toCvCopy(depth);
    img_depth_ = cv_depth_ptr->image;
    Matrix<double> matrix_depth(info->width, info->height);

    for (int r = 0;r < 480;r++){
        for (int c = 0;c < 640;c++) {
            matrix_depth(c, r) = img_depth_.at<float>(r,c);
        }
    }

    // Generate base camera
    Matrix<double> R = Eye<double>(3);
    Vector<double> t(3, 0.0);
    Vector<double> GP(0.0, 0.99, 0.0, 1.8); // just some placeholders which will be replaced after GP Estimation
    Matrix<double> K(3,3, (double*)&info->K[0]);

    Camera camera(K,R,t,GP);
    PointCloud point_cloud(camera, matrix_depth);


    GP = GPEstimator.ComputeGroundPlane(point_cloud);

    // Generate Ground Plane Message
    ground_plane_estimation::GroundPlane ground_plane_msg;
    ground_plane_msg.header = depth->header;
    ground_plane_msg.d = GP(3);
    ground_plane_msg.n.push_back(GP(0));
    ground_plane_msg.n.push_back(GP(1));
    ground_plane_msg.n.push_back(GP(2));

    pub_ground_plane.publish(ground_plane_msg);

}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void connectCallback(message_filters::Subscriber<CameraInfo> &sub_cam,
                     image_transport::SubscriberFilter &sub_depth,
                     image_transport::ImageTransport &it){
    if(!pub_ground_plane.getNumSubscribers()) {
        ROS_DEBUG("Ground Plane estimated: No subscribers. Unsubscribing.");
        sub_cam.unsubscribe();
        sub_depth.unsubscribe();
    } else {
        ROS_DEBUG("Ground Plane estimated: New subscribers. Subscribing.");
        sub_cam.subscribe();
        sub_depth.subscribe(it,sub_depth.getTopic().c_str(),1);
    }
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "ground_plane");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    int queue_size;
    string cam_ns;
    string pub_topic_gp;
    string topic_depth_image;
    string topic_camera_info;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("queue_size", queue_size, int(5));

    private_node_handle_.param("camera_namespace", cam_ns, string("/head_xtion"));
    private_node_handle_.param("depth_image", topic_depth_image, string("/depth/image_rect"));
    topic_depth_image = cam_ns + topic_depth_image;
    private_node_handle_.param("camera_info_rgb", topic_camera_info, string("/rgb/camera_info"));
    topic_camera_info = cam_ns + topic_camera_info;

    ROS_DEBUG("ground_plane: Queue size for synchronisation is set to: %i", queue_size);

    // Image transport handle
    image_transport::ImageTransport it(private_node_handle_);

    // Create a subscriber.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    image_transport::SubscriberFilter subscriber_depth;
    subscriber_depth.subscribe(it, topic_depth_image.c_str(), 1); subscriber_depth.unsubscribe();
    message_filters::Subscriber<CameraInfo> subscriber_camera_info(n, topic_camera_info.c_str(), 1); subscriber_camera_info.unsubscribe();

    ros::SubscriberStatusCallback con_cb = boost::bind(&connectCallback,
                                                       boost::ref(subscriber_camera_info),
                                                       boost::ref(subscriber_depth),
                                                       boost::ref(it));

    //The real queue size for synchronisation is set here.
    sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy(queue_size);
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.

    if(!ReadConfigParams(boost::ref(n))) return 1;

    const sync_policies::ApproximateTime<Image, CameraInfo> MyConstSyncPolicy = MySyncPolicy;

    Synchronizer< sync_policies::ApproximateTime<Image, CameraInfo> > sync(MyConstSyncPolicy,
                                                                                  subscriber_depth, subscriber_camera_info);

    sync.registerCallback(boost::bind(&callback, _1, _2));


    // Create a topic publisher
    private_node_handle_.param("ground_plane", pub_topic_gp, string("/ground_plane"));
    pub_ground_plane = n.advertise<ground_plane_estimation::GroundPlane>(pub_topic_gp.c_str(), 10, con_cb, con_cb);


    ros::spin();

    return 0;
}


