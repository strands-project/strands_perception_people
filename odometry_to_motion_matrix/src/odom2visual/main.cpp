// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Geometry>

#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include "visual_odometry/VisualOdometry.h"

ros::Publisher pub_message;

void callback(const nav_msgs::OdometryConstPtr& odom) {
    Eigen::Affine3d eigenTr;
    tf::poseMsgToEigen(odom->pose.pose, eigenTr);
    Eigen::Matrix4d m = eigenTr.matrix().inverse();

    visual_odometry::VisualOdometry fovis_info_msg;
    fovis_info_msg.header = odom->header;
    fovis_info_msg.motion_estimate_valid = true;
    fovis_info_msg.transformation_matrix.resize(4*4);
    for(int i = 0; i < 4*4; i++)
        fovis_info_msg.transformation_matrix[i] = m.data()[i];
     pub_message.publish(fovis_info_msg);
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void connectCallback(ros::Subscriber &sub_odom,
                     ros::NodeHandle &n,
                     std::string odom_topic){
    if(!pub_message.getNumSubscribers()) {
        ROS_DEBUG("Odom: No subscribers. Unsubscribing.");
        sub_odom.shutdown();
    } else {
        ROS_DEBUG("Odom: New subscribers. Subscribing.");
        sub_odom = n.subscribe(odom_topic.c_str(), 1, &callback);
    }
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "odom2visual");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    std::string odom_topic;
    std::string pub_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("odom", odom_topic, std::string("/odom"));

    // Create a subscriber.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    ros::Subscriber sub_odom; //Subscribers have to be defined out of the if scope to have effect.

    ros::SubscriberStatusCallback con_cb = boost::bind(&connectCallback,
                                                       boost::ref(sub_odom),
                                                       boost::ref(n),
                                                       odom_topic);

    private_node_handle_.param("motion_parameters", pub_topic, std::string("/visual_odometry/motion_matrix"));
    pub_message = n.advertise<visual_odometry::VisualOdometry>(pub_topic.c_str(), 10, con_cb, con_cb);

    ros::spin();

    return 0;
}
