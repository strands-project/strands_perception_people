// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "visual_odometry/VisualOdometry.h"

#include "string.h"
#include "boost/thread.hpp"

#include "fovis.hpp"
#include <iostream>
#include <fstream>




using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

//sensor_msgs::CameraInfo* camera_info = NULL;
//boost::mutex camera_info_mutex;
/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
ros::Publisher pub_message;
fovis::VisualOdometry* odom = NULL;
fovis::CameraIntrinsicsParameters cam_params;

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

        ofstream aStream("/home/mitzel/Desktop/depth.txt");
        for (int i = 0; i < depth->width*depth->height; i++)
        {
            aStream << (double) depth->data[i] << endl;
        }
        ROS_INFO("Created: /home/mitzel/Desktop/depth.txt");
    }

    ros::WallTime start_time = ros::WallTime::now();
    fovis::DepthImage* fv_dp = new fovis::DepthImage(cam_params,info->width,info->height);
    fv_dp->setDepthImage((float*) &depth->data[0]);

    odom->processFrame(&image->data[0], fv_dp);

    Eigen::Isometry3d cam_to_local = odom->getPose();
    Eigen::Matrix4d m1 = cam_to_local.matrix().inverse();

    visual_odometry::VisualOdometry fovis_info_msg;
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
    string topic_image_mono;
    string topic_depth_image;
    string topic_camera_info;
    string pub_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("mono_image", topic_image_mono, string("/camera/rgb/image_rect"));
    private_node_handle_.param("depth_image", topic_depth_image, string("/camera/depth/image_rect"));
    private_node_handle_.param("camera_info", topic_camera_info, string("/camera/rgb/camera_info"));

    // Create a subscriber.
    message_filters::Subscriber<Image> subscriber_mono(n, topic_image_mono.c_str(), 50);
    message_filters::Subscriber<Image> subscriber_depth(n, topic_depth_image.c_str(), 50);
    message_filters::Subscriber<CameraInfo> subscriber_camera_info(n, topic_camera_info.c_str(), 50);

    sync_policies::ApproximateTime<Image, Image, CameraInfo> MySyncPolicy(10);
    MySyncPolicy.setAgePenalty(10);

    const sync_policies::ApproximateTime<Image, Image, CameraInfo> MyConstSyncPolicy = MySyncPolicy;

    Synchronizer< sync_policies::ApproximateTime<Image, Image, CameraInfo> > sync(MyConstSyncPolicy,
                                                                                  subscriber_mono, subscriber_depth, subscriber_camera_info);

    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    // Create a topic publisher
    private_node_handle_.param("motion_parameters", pub_topic, string("/visual_odometry/motion_matrix"));
    pub_message = n.advertise<visual_odometry::VisualOdometry>(pub_topic.c_str(), 10);

    ros::spin();

    return 0;
}

