// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include <boost/thread.hpp>

#include "fovis.hpp"

#include "visual_odometry/VisualOdometry.h"



using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace visual_odometry;

ros::Publisher pub_message;
fovis::VisualOdometry* odom = NULL;
fovis::CameraIntrinsicsParameters cam_params;

cv::Mat img_depth_;
cv_bridge::CvImagePtr cv_depth_ptr;	// cv_bridge for depth image

bool recover = false;
bool first = true;

void callback(const ImageConstPtr &image, const ImageConstPtr &depth, const CameraInfoConstPtr &info)
{
    // Initialising odomometry at first callback call or for recovery
    if(odom == NULL)
    {
        // Only needs to be done once at startup and not for recovery
        if(first) {
            first = false;
            memset(&cam_params, 0, sizeof(fovis::CameraIntrinsicsParameters));
            cam_params.width = info->width;
            cam_params.height = info->height;
            cam_params.fx = info->K[0];
            cam_params.fy = info->K[4];
            cam_params.cx = info->K[2];
            cam_params.cy = info->K[5];
        }

        fovis::Rectification* fovis_rect;

        fovis_rect = new fovis::Rectification(cam_params);
        fovis::VisualOdometryOptions options = fovis::VisualOdometry::getDefaultOptions();
        odom = new fovis::VisualOdometry(fovis_rect, options);
    }

    cv_depth_ptr = cv_bridge::toCvCopy(depth);
    img_depth_ = cv_depth_ptr->image;

    ros::WallTime start_time = ros::WallTime::now();
    fovis::DepthImage* fv_dp = new fovis::DepthImage(cam_params,info->width,info->height);
    fv_dp->setDepthImage((float*)(img_depth_.data));

    odom->processFrame(&image->data[0], fv_dp);

    Eigen::Isometry3d cam_to_local = odom->getPose();
    Eigen::Matrix4d m1 = cam_to_local.matrix().inverse();

    VisualOdometry fovis_info_msg;
    fovis_info_msg.header = image->header;

    fovis_info_msg.change_reference_frame = odom->getChangeReferenceFrames();
    fovis_info_msg.fast_threshold = odom->getFastThreshold();
    const fovis::OdometryFrame* frame = odom->getTargetFrame();
    fovis_info_msg.num_total_detected_keypoints = frame->getNumDetectedKeypoints();
    fovis_info_msg.num_total_keypoints = frame->getNumKeypoints();
    fovis_info_msg.num_detected_keypoints.resize(frame->getNumLevels());
    fovis_info_msg.num_keypoints.resize(frame->getNumLevels());
    for (int i = 0; i < frame->getNumLevels(); ++i)
    {
        fovis_info_msg.num_detected_keypoints[i] = frame->getLevel(i)->getNumDetectedKeypoints();
        fovis_info_msg.num_keypoints[i] = frame->getLevel(i)->getNumKeypoints();
    }
    const fovis::MotionEstimator* estimator = odom->getMotionEstimator();
    fovis_info_msg.motion_estimate_status_code = estimator->getMotionEstimateStatus();
    fovis_info_msg.motion_estimate_status = fovis::MotionEstimateStatusCodeStrings[fovis_info_msg.motion_estimate_status_code];
    fovis_info_msg.num_matches = estimator->getNumMatches();
    fovis_info_msg.num_inliers = estimator->getNumInliers();
    fovis_info_msg.num_reprojection_failures = estimator->getNumReprojectionFailures();
    fovis_info_msg.motion_estimate_valid = estimator->isMotionEstimateValid();
    ros::WallDuration time_elapsed = ros::WallTime::now() - start_time;
    fovis_info_msg.runtime = time_elapsed.toSec();


    fovis_info_msg.transformation_matrix.resize(4*4);
    for(int i = 0; i < 4*4; i++)
    {
        fovis_info_msg.transformation_matrix[i] = m1.data()[i];
        if(isnan(m1.data()[i])) { // Detected nan values and trigger recovery
            recover = true;
        }
    }

    pub_message.publish(fovis_info_msg);
    delete fv_dp;

    // delete odometry and reset at next callback call.
    if(recover) {
        ROS_WARN("Detected 'nan' values in motion matrix. Will try to auto recover by resetting visual odometry.");
        recover = false;
        delete odom;
        odom = NULL;
    }
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void connectCallback(message_filters::Subscriber<CameraInfo> &sub_cam,
                     image_transport::SubscriberFilter &sub_mon,
                     image_transport::SubscriberFilter &sub_dep,
                     image_transport::ImageTransport &it){
    if(!pub_message.getNumSubscribers()) {
        ROS_DEBUG("Visual Odometry: No subscribers. Unsubscribing.");
        sub_cam.unsubscribe();
        sub_mon.unsubscribe();
        sub_dep.unsubscribe();
    } else {
        ROS_DEBUG("Visual Odometry: New subscribers. Subscribing.");
        sub_cam.subscribe();
        sub_mon.subscribe(it,sub_mon.getTopic().c_str(),1);
        sub_dep.subscribe(it,sub_dep.getTopic().c_str(),1);
    }
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "visual_odometry");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    int queue_size;
    string cam_ns;
    string pub_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("queue_size", queue_size, int(5));
    private_node_handle_.param("camera_namespace", cam_ns, string("/head_xtion"));

    string topic_image_mono;
    private_node_handle_.param("mono_image", topic_image_mono, string("/rgb/image_mono"));
    topic_image_mono = cam_ns + topic_image_mono;
    string topic_camera_info;
    private_node_handle_.param("camera_info_depth", topic_camera_info, string("/depth/camera_info"));
    topic_camera_info = cam_ns + topic_camera_info;
    string topic_depth_image;
    private_node_handle_.param("depth_image", topic_depth_image, string("/depth/image_rect"));
    topic_depth_image = cam_ns + topic_depth_image;

    ROS_DEBUG("visual_odometry: Queue size for synchronisation is set to: %i", queue_size);

    // Image transport handle
    image_transport::ImageTransport it(private_node_handle_);

    // Create a subscriber.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    image_transport::SubscriberFilter subscriber_mono;
    subscriber_mono.subscribe(it, topic_image_mono.c_str(), 1); subscriber_mono.unsubscribe();
    image_transport::SubscriberFilter subscriber_depth;
    subscriber_depth.subscribe(it, topic_depth_image.c_str(), 1); subscriber_depth.unsubscribe();
    Subscriber<CameraInfo> subscriber_camera_info(n, topic_camera_info.c_str(), 1); subscriber_camera_info.unsubscribe();

    ros::SubscriberStatusCallback con_cb = boost::bind(&connectCallback,
                                                       boost::ref(subscriber_camera_info),
                                                       boost::ref(subscriber_mono),
                                                       boost::ref(subscriber_depth),
                                                       boost::ref(it));

    //The real queue size for synchronisation is set here.
    sync_policies::ApproximateTime<Image, Image, CameraInfo> MySyncPolicy(queue_size);
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.

    const sync_policies::ApproximateTime<Image, Image, CameraInfo> MyConstSyncPolicy = MySyncPolicy;

    Synchronizer< sync_policies::ApproximateTime<Image, Image, CameraInfo> > sync(MyConstSyncPolicy,
                                                                                  subscriber_mono, subscriber_depth, subscriber_camera_info);

    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    // Create a topic publisher
    private_node_handle_.param("motion_parameters", pub_topic, string("/visual_odometry/motion_matrix"));
    pub_message = n.advertise<VisualOdometry>(pub_topic.c_str(), 10, con_cb, con_cb);

    ros::spin();

    return 0;
}

