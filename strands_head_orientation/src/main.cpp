#include <iostream>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>

#include "warco.hpp"
#include "strands_perception_people_msgs/UpperBodyDetector.h"
#include "strands_perception_people_msgs/HeadOrientations.h"

// For UpperBodyDetector and HeadOrientations
using namespace strands_perception_people_msgs;

warco::Warco* g_model;
ros::Publisher g_pub;

void cb(const sensor_msgs::ImageConstPtr &color, const UpperBodyDetector::ConstPtr &upper)
{
    // No subscriber is interested, don't do anything.
    if(g_pub.getNumSubscribers() == 0) {
        return;
    }

    // No upper body detections, don't do anything.
    size_t ndetects = upper->pos_x.size();
    if(ndetects == 0) {
        return;
    }

    ROS_DEBUG("Entering callback of heads, and it got subscribers and detections.");

    // Get the OpenCV image out of the ros message. No-copy.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(color);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // TODO: Do something.
    //cv::imshow("foo", cv_ptr->image);

    // Publish the result.
    HeadOrientations msg;
    msg.header = color->header;

    for(size_t i = 0 ; i < ndetects ; ++i) {
        cv::Rect bbox(upper->pos_x[i], upper->pos_y[i], upper->width[i], upper->height[i]);
        std::cout << bbox << std::endl;

        // Ugly as fuck hardcoding. Should load from file.
        // front = 0.f, left = 90.f, right = -90.f, back = 180.f, background = nan
        switch(g_model->predict_proba(cv::Mat(cv_ptr->image, bbox))) {
        case 0: msg.angles.push_back(0.f); msg.confidences.push_back(0.83f); break;
        case 1: msg.angles.push_back(90.f); msg.confidences.push_back(0.83f); break;
        case 2: msg.angles.push_back(-90.f); msg.confidences.push_back(0.83f); break;
        case 3: msg.angles.push_back(180.f); msg.confidences.push_back(0.83f); break;
        case 4: msg.angles.push_back(nan("")); msg.confidences.push_back(0.83f); break;
        default: msg.angles.push_back(0.f); msg.confidences.push_back(0.0f); break;
        }
    }

    g_pub.publish(msg);
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "head_orientation");
    ros::NodeHandle nh;

    // Declare variables that can be modified by launch file (or command line?)
    int queue_size;
    std::string topic_upperbody;
    std::string cam_ns;
    std::string pub_topic;
    std::string model_dir;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be
    // run simultaneously while using different parameters.
    ros::NodeHandle nh_("~");
    nh_.param("queue_size", queue_size, int(10));
    nh_.param("camera_namespace", cam_ns, std::string("/head_xtion"));
    nh_.param("upper_body_detections", topic_upperbody, std::string("/upper_body_detector/detections"));
    nh_.param("head_ori", pub_topic, std::string("/head_orientation/head_ori"));
    nh_.param("model_dir", model_dir, std::string(""));
    std::string topic_color_image = cam_ns + "/rgb/image_color";

    if(model_dir.empty()) {
        ROS_ERROR("No model file specified.");
        ROS_ERROR("Run with: rosrun strands_head_orientation strands_head_orientation _model_dir:=/path/to/model");
        exit(1);
    }

    ROS_DEBUG("head_orientation: Sync queue size is set to: %i", queue_size);

    ROS_INFO("Loading model folder %s", model_dir.c_str());
    g_model = new warco::Warco(model_dir);

    // Image transport is optimized for, well, pub/sub images.
    image_transport::ImageTransport it(nh_);

    // Create a subscriber.
    // Set queue size to 1 because generating a queue here will only pile up
    // images and delay the output by the amount of queued images.
    image_transport::SubscriberFilter sub_color(it, topic_color_image.c_str(), 1);

    message_filters::Subscriber<UpperBodyDetector> sub_upperbody(nh, topic_upperbody.c_str(), 1);

    // Create a synchronized callback for both the RGB image and the upper body
    // detections so one callback gets called with both messages together.
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, UpperBodyDetector> SyncType;
    const SyncType sync_policy(queue_size);

    message_filters::Synchronizer<SyncType> mysync(sync_policy, sub_color, sub_upperbody);
    mysync.registerCallback(boost::bind(&cb, _1, _2));

    // Create a topic publisher
    g_pub = nh.advertise<HeadOrientations>(pub_topic.c_str(), 10);

    ros::spin();

    delete g_model;
    return 0;
}

