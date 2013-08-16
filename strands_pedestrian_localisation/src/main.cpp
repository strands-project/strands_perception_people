#include <ros/ros.h>
#include <ros/time.h>

#include <string.h>

#include "strands_perception_people_msgs/PedestrianTracking.h"
#include "strands_perception_people_msgs/PedestrianTrackingArray.h"
#include "strands_perception_people_msgs/UpperBodyDetector.h"

using namespace std;
using namespace strands_perception_people_msgs;

ros::Publisher pub_message;

void trackingCallback(const PedestrianTrackingArray::ConstPtr &pta)
{
    ROS_INFO("Entered pta callback");
    for(int i = 0; i < pta->pedestrians.size(); i++) {
        PedestrianTracking pt = pta->pedestrians[i];
        if(pt.traj_x.size() && pt.traj_y.size() && pt.traj_z.size())
            ROS_INFO("Position tracking x: %f, y: %f, z: %f", pt.traj_x[0], pt.traj_y[0], pt.traj_z[0]);
//        else
//            ROS_WARN("pt is zero");
    }
    //TODO: Fill with magic
}

void ubdCallback(const UpperBodyDetector::ConstPtr &msg)
{
    ROS_INFO("Entered ubd callback: %i", msg->pos_x.size());
    for(int i = 0; i < msg->pos_x.size(); i++) {
        if(msg->pos_x.size()>i && msg->pos_y.size()>i && msg->dist.size()>i) {
            ROS_INFO("Position ubd x: %i, y: %i, z: %i", (int)msg->pos_x[i], (int)msg->pos_y[i], (int)msg->dist[i]);
        }
    }
    //TODO: Fill with magic
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "pedestrian_localisation");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    string pta_topic;
    string ubd_topic;
    string pub_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("pedestrian_array", pta_topic, string("/pedestrian_tracking/pedestrian_array"));
    private_node_handle_.param("upper_body_detections", ubd_topic, string("/upper_body_detector/detections"));

    // Create a subscriber.
    ros::Subscriber pta_sub = n.subscribe(pta_topic.c_str(), 10, &trackingCallback);
    ros::Subscriber ubd_sub = n.subscribe(ubd_topic.c_str(), 10, &ubdCallback);

    private_node_handle_.param("localisations", pub_topic, string("/pedestrian_localisation/localisations"));
//    pub_message = n.advertise<strands_perception_people_msgs::UpperBodyDetector>(pub_topic.c_str(), 10);

    ros::spin();
    return 0;
}


