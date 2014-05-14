#include <iostream>
#include <random>
#include <sstream>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// For the node
#include "strands_perception_people_msgs/UpperBodyDetector.h"

// For the db
#include "ros_datacentre/message_store.h"

// For the service
#include "strands_perception_people_msgs/StartHeadAnalysis.h"
#include "strands_perception_people_msgs/StopHeadAnalysis.h"
#include "strands_perception_people_msgs/IsHeadAnalysisRunning.h"

// For UpperBodyDetector
using namespace strands_perception_people_msgs;

// WHY CAN'T I HOLD ALL THESE GLOBALS?
bool g_running = false;
ros_datacentre::MessageStoreProxy* g_messageStore = 0;
static const double g_pWholeImageIfDetection = 0.05;
static const double g_pWholeImageIfNoDetection = 0.01;

bool doWithProb(double p)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0, 1);
    return g_running ? dis(gen) < p : false;
}

template<typename T>
std::string to_s(const T& v)
{
    std::ostringstream ss;
    ss << v;
    return ss.str();
}

void cb(const sensor_msgs::ImageConstPtr &color, const UpperBodyDetector::ConstPtr &upper)
{
    // THOU SHALL NOT RUN
    if(! g_running)
        return;

    size_t ndetects = upper->pos_x.size();

    // Randomly store a whole image.
    if(g_messageStore && ndetects == 0 && doWithProb(g_pWholeImageIfNoDetection))
        std::cout << "Inserted full_col_nodet_" << color->header.seq << " as " <<
            g_messageStore->insertNamed("full_col_nodet_" + to_s(color->header.seq), *color)
            << std::endl;
    if(g_messageStore && ndetects > 0 && doWithProb(g_pWholeImageIfDetection))
        std::cout << "Inserted full_col_det_" << color->header.seq << " as " <<
            g_messageStore->insertNamed("full_col_det_" + to_s(color->header.seq), *color)
            << std::endl;

    // No upper body detections, don't do anything else.
    if(ndetects == 0)
        return;

    ROS_DEBUG("Entering callback of heads, and it is awake and got detections.");

    // Get the OpenCV image out of the ros message. No-copy.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(color);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    for(size_t i = 0 ; i < ndetects ; ++i) {
        cv::Rect bbox(upper->pos_x[i], upper->pos_y[i], upper->width[i], upper->height[i]);

        // Cut it to the image. It might be outside if people are only half-on-screen.
        bbox &= cv::Rect(0, 0, cv_ptr->image.cols, cv_ptr->image.rows);
        ROS_INFO_STREAM("Checking head in " << bbox);

        // Do store the stuff in the database for later debugging.
        // Cloning the matrix because else it keeps pointing to the original
        // big one and just increases stride, effectively storing too much in
        // the database.
        if(g_messageStore && g_running)
            std::cout << "Inserted upper_col" << color->header.seq << "_" << i << " as " <<
                g_messageStore->insertNamed("upper_col_" + to_s(color->header.seq) + "_" + to_s(i),
                    cv_bridge::CvImage(color->header, color->encoding, cv::Mat(cv_ptr->image, bbox).clone()))
                << std::endl;
    }
}

bool cbStart(StartHeadAnalysis::Request& req, StartHeadAnalysis::Response& res)
{
    res.was_running = g_running;
    if(g_running)
        ROS_INFO("Already recording.");
    else
        ROS_INFO("Starting to record heads.");
    g_running = true;

    return true;
}

bool cbStop(StopHeadAnalysis::Request& req, StopHeadAnalysis::Response& res)
{
    res.was_running = g_running;
    if(! g_running)
        ROS_INFO("Ain't even recording.");
    else
        ROS_INFO("Stopping to record heads.");
    g_running = false;

    return true;
}

bool cbIsRecording(IsHeadAnalysisRunning::Request& req, IsHeadAnalysisRunning::Response& res)
{
    res.is_running = g_running;
    return true;
}


int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "head_orientation_db");
    ros::NodeHandle nh;

    // Declare variables that can be modified by launch file (or command line?)
    int queue_size;
    std::string topic_upperbody;
    std::string cam_ns;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be
    // run simultaneously while using different parameters.
    ros::NodeHandle nh_("~");
    nh_.param("queue_size", queue_size, int(5));
    nh_.param("camera_namespace", cam_ns, std::string("/head_xtion"));
    nh_.param("upper_body_detections", topic_upperbody, std::string("/upper_body_detector/detections"));
    nh_.param("autostart", g_running, true);
    std::string topic_color_image = cam_ns + "/rgb/image_rect_color";

    ROS_DEBUG("head_orientation: Sync queue size is set to: %i", queue_size);

    if(g_running) {
        ROS_INFO("Starting to store upper body detections right away.");
    } else {
        ROS_INFO("Waiting for a start signal before storing. (Read the README!)");
    }

    // we will store our results in a separate collection.
    g_messageStore = new ros_datacentre::MessageStoreProxy(nh_, "heads");

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

    // Advertise a service for starting/stopping.
    ros::ServiceServer srv_start = nh.advertiseService("start_head_recording", cbStart);
    ros::ServiceServer srv_stop = nh.advertiseService("stop_head_recording", cbStop);
    ros::ServiceServer srv_get = nh.advertiseService("status_head_recording", cbIsRecording);

    ros::spin();

    delete g_messageStore;
    return 0;
}

