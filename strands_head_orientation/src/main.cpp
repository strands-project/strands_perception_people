#include <iostream>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <opencv_warco/warco.hpp>

// For the node
#include "upper_body_detector/UpperBodyDetector.h"
#include "strands_head_orientation/HeadOrientations.h"

// For the service
#include "strands_head_orientation/StartHeadAnalysis.h"
#include "strands_head_orientation/StopHeadAnalysis.h"
#include "strands_head_orientation/IsHeadAnalysisRunning.h"

// For UpperBodyDetector and HeadOrientations
using namespace strands_head_orientation;
using namespace upper_body_detector;

// WHY CAN'T I HOLD ALL THESE GLOBALS?
bool g_running = false;
warco::Warco* g_model;
ros::Publisher g_pub;
image_transport::Publisher g_dbgpub;

// Brett Styles greets you.
void visualize(cv::Mat& inout, cv::Rect bbox, unsigned pred, double conf)
{
    // Bounding-box with corresponding color.
    cv::Scalar col = cv::Scalar(187,226,188);
    if(3 < pred)
        col = cv::Scalar(187,188,226);

    cv::rectangle(inout, bbox, col, 5);

    // "Arrow" showing the orientation. See caller below for details.
    cv::Point center = (bbox.tl() + bbox.br())*0.5;
    switch(pred) {
    case 0: cv::line(inout, center, cv::Point(bbox.x + bbox.width/2, bbox.y + bbox.height  ), col, 5, CV_AA); break;
    case 1: cv::line(inout, center, cv::Point(bbox.x               , bbox.y + bbox.height/2), col, 5, CV_AA); break;
    case 2: cv::line(inout, center, cv::Point(bbox.x + bbox.width  , bbox.y + bbox.height/2), col, 5, CV_AA); break;
    case 3: cv::line(inout, center, cv::Point(bbox.x + bbox.width/2, bbox.y                ), col, 5, CV_AA); break;
    case 4: default: break;
    }
}

void cb(const sensor_msgs::ImageConstPtr &color, const UpperBodyDetector::ConstPtr &upper)
{
    // THOU SHALL NOT RUN
    if(! g_running)
        return;

    // No subscriber is interested, don't do anything.
    if(g_pub.getNumSubscribers() + g_dbgpub.getNumSubscribers() == 0) {
        return;
    }

    bool dbg = g_dbgpub.getNumSubscribers();

    // No upper body detections, don't do anything.
    size_t ndetects = upper->pos_x.size();
    if(ndetects == 0) {
        return;
    }

    ROS_DEBUG("Entering callback of heads, and it got subscribers and detections.");

    // Get the OpenCV image out of the ros message. No-copy.
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_bridge::CvImagePtr cv_dbg;
    try {
        cv_ptr = cv_bridge::toCvShare(color);
        if(dbg) cv_dbg = cv_bridge::toCvCopy(color, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Publish the result.
    HeadOrientations msg;
    msg.header = color->header;

    for(size_t i = 0 ; i < ndetects ; ++i) {
        cv::Rect bbox(upper->pos_x[i], upper->pos_y[i], upper->width[i], upper->height[i]);

        // Cut it to the image. It might be outside if people are only half-on-screen.
        bbox &= cv::Rect(0, 0, cv_ptr->image.cols, cv_ptr->image.rows);
        ROS_INFO_STREAM("Checking head in " << bbox);

        // Ugly as fuck hardcoding. Should load from file.
        // front = 0.f, left = 90.f, right = -90.f, back = 180.f, background = nan
        unsigned pred = g_model->predict_proba(cv::Mat(cv_ptr->image, bbox));
        switch(pred) {
        case 0: msg.angles.push_back(0.f); break;
        case 1: msg.angles.push_back(90.f); break;
        case 2: msg.angles.push_back(-90.f); break;
        case 3: msg.angles.push_back(180.f); break;
        case 4: msg.angles.push_back(nan("")); break;
        default:
            ROS_ERROR("Got invalid prediction. What model are you loading?");
            return;
        }
        msg.confidences.push_back(0.83);

        if(dbg) visualize(cv_dbg->image, bbox, pred, 0.83);
    }

    g_pub.publish(msg);

    // Create a debug image for visualizing.
    if(dbg)
        try {
            g_dbgpub.publish(cv_dbg->toImageMsg());
        } catch(const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
}

bool cbStartAnalyzing(StartHeadAnalysis::Request& req, StartHeadAnalysis::Response& res)
{
    res.was_running = g_running;
    if(g_running)
        ROS_INFO("Already running.");
    else
        ROS_INFO("Starting to analyze heads.");
    g_running = true;

    return true;
}

bool cbStopAnalyzing(StopHeadAnalysis::Request& req, StopHeadAnalysis::Response& res)
{
    res.was_running = g_running;
    if(! g_running)
        ROS_INFO("Ain't even running.");
    else
        ROS_INFO("Stopping to analyze heads.");
    g_running = false;

    return true;
}

bool cbIsHeadAnalysisRunning(IsHeadAnalysisRunning::Request& req, IsHeadAnalysisRunning::Response& res)
{
    res.is_running = g_running;
    return true;
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
    nh_.param("autostart", g_running, true);
    std::string topic_color_image = cam_ns + "/rgb/image_rect_color";

    if(model_dir.empty()) {
        ROS_ERROR("No model file specified.");
        ROS_ERROR("Run with: rosrun strands_head_orientation strands_head_orientation _model_dir:=/path/to/model");
        exit(1);
    }

    ROS_DEBUG("head_orientation: Sync queue size is set to: %i", queue_size);

    ROS_INFO("Loading model folder %s", model_dir.c_str());
    g_model = new warco::Warco(model_dir);
    ROS_INFO("Done loading");

    if(g_running) {
        ROS_INFO("Starting to analyze upper body detections right away.");
    } else {
        ROS_INFO("Waiting for a start signal before analyzing. (Read the README!)");
    }

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

    g_dbgpub = it.advertise("/head_orientation/image", 1);

    // Advertise a service for starting/stopping.
    ros::ServiceServer srv_start = nh.advertiseService("start_head_analysis", cbStartAnalyzing);
    ros::ServiceServer srv_stop = nh.advertiseService("stop_head_analysis", cbStopAnalyzing);
    ros::ServiceServer srv_get = nh.advertiseService("status_head_analysis", cbIsHeadAnalysisRunning);

    ros::spin();

    delete g_model;
    return 0;
}

