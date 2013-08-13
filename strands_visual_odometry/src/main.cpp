// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <string.h>
#include <boost/thread.hpp>

#include "fovis.hpp"
#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>

#include "strands_perception_people_msgs/VisualOdometry.h"



using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace strands_perception_people_msgs;

//sensor_msgs::CameraInfo* camera_info = NULL;
//boost::mutex camera_info_mutex;
/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
ros::Publisher pub_message;
fovis::VisualOdometry* odom = NULL;
fovis::CameraIntrinsicsParameters cam_params;

cv::Mat img_depth_;
cv_bridge::CvImagePtr cv_depth_ptr;	// cv_bridge for depth image

void callback(const ImageConstPtr &image, const ImageConstPtr &depth, const CameraInfoConstPtr &info)
{

    if(odom==NULL)
    {
        memset(&cam_params, 0, sizeof(fovis::CameraIntrinsicsParameters));
        cam_params.width = info->width;
        cam_params.height = info->height;
        cam_params.fx = info->K[0];
        cam_params.fy = info->K[4];
        cam_params.cx = info->K[2];
        cam_params.cy = info->K[5];

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

    fovis_info_msg.change_reference_frame =
            odom->getChangeReferenceFrames();
    fovis_info_msg.fast_threshold =
            odom->getFastThreshold();
    const fovis::OdometryFrame* frame =
            odom->getTargetFrame();
    fovis_info_msg.num_total_detected_keypoints =
            frame->getNumDetectedKeypoints();
    fovis_info_msg.num_total_keypoints = frame->getNumKeypoints();
    fovis_info_msg.num_detected_keypoints.resize(frame->getNumLevels());
    fovis_info_msg.num_keypoints.resize(frame->getNumLevels());
    for (int i = 0; i < frame->getNumLevels(); ++i)
    {
        fovis_info_msg.num_detected_keypoints[i] =
                frame->getLevel(i)->getNumDetectedKeypoints();
        fovis_info_msg.num_keypoints[i] =
                frame->getLevel(i)->getNumKeypoints();
    }
    const fovis::MotionEstimator* estimator =
            odom->getMotionEstimator();
    fovis_info_msg.motion_estimate_status_code =
            estimator->getMotionEstimateStatus();
    fovis_info_msg.motion_estimate_status =
            fovis::MotionEstimateStatusCodeStrings[
            fovis_info_msg.motion_estimate_status_code];
    fovis_info_msg.num_matches = estimator->getNumMatches();
    fovis_info_msg.num_inliers = estimator->getNumInliers();
    fovis_info_msg.num_reprojection_failures =
            estimator->getNumReprojectionFailures();
    fovis_info_msg.motion_estimate_valid =
            estimator->isMotionEstimateValid();
    ros::WallDuration time_elapsed = ros::WallTime::now() - start_time;
    fovis_info_msg.runtime = time_elapsed.toSec();


    fovis_info_msg.transformation_matrix.resize(4*4);
    for(int i = 0; i < 4*4; i++)
    {
        fovis_info_msg.transformation_matrix[i] = m1.data()[i];
    }

    pub_message.publish(fovis_info_msg);
    delete fv_dp;
}


int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "visual_odometry");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    int queue_size;
    string topic_image_mono;
    string topic_depth_image;
    string topic_camera_info;
    string pub_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("queue_size", queue_size, int(5));
    private_node_handle_.param("mono_image", topic_image_mono, string("/camera/rgb/image_mono"));
    private_node_handle_.param("depth_image", topic_depth_image, string("/camera/depth/image"));
    private_node_handle_.param("camera_info", topic_camera_info, string("/camera/rgb/camera_info"));

    ROS_DEBUG("visual_odometry: Queue size for synchronisation is set to: %i", queue_size);

    // Create a subscriber.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    Subscriber<Image> subscriber_mono(n, topic_image_mono.c_str(), 1);
    Subscriber<Image> subscriber_depth(n, topic_depth_image.c_str(), 1);
    Subscriber<CameraInfo> subscriber_camera_info(n, topic_camera_info.c_str(), 1);

    //The real queue size for synchronisation is set here.
    sync_policies::ApproximateTime<Image, Image, CameraInfo> MySyncPolicy(queue_size);
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.

    const sync_policies::ApproximateTime<Image, Image, CameraInfo> MyConstSyncPolicy = MySyncPolicy;

    Synchronizer< sync_policies::ApproximateTime<Image, Image, CameraInfo> > sync(MyConstSyncPolicy,
                                                                                  subscriber_mono, subscriber_depth, subscriber_camera_info);

    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    // Create a topic publisher
    private_node_handle_.param("motion_parameters", pub_topic, string("/visual_odometry/motion_matrix"));
    pub_message = n.advertise<VisualOdometry>(pub_topic.c_str(), 10);

    ros::spin();

    return 0;
}

