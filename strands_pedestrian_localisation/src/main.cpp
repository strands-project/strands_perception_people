#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <string.h>

#include "strands_perception_people_msgs/PedestrianTracking.h"
#include "strands_perception_people_msgs/PedestrianTrackingArray.h"
#include "strands_perception_people_msgs/UpperBodyDetector.h"

using namespace std;
using namespace strands_perception_people_msgs;

ros::Publisher pub_message;
tf::TransformListener* listener;

void trackingCallback(const PedestrianTrackingArray::ConstPtr &pta)
{
    for(int i = 0; i < pta->pedestrians.size(); i++) {
        PedestrianTracking pt = pta->pedestrians[i];
        if(pt.traj_x.size() && pt.traj_y.size() && pt.traj_z.size()) {
            ROS_DEBUG("pta: Position world x: %f, y: %f, z: %f",
                     pt.traj_x[0],
                     pt.traj_y[0],
                     pt.traj_z[0]);
            ROS_DEBUG("pta: Position cam x: %f, y: %f, z: %f",
                     pt.traj_x_camera[0],
                     pt.traj_y_camera[0],
                     pt.traj_z_camera[0]);

            //Create stamped pose for tf
            geometry_msgs::PoseStamped poseInCamCoords;
            geometry_msgs::PoseStamped poseInRobotCoords;
            poseInCamCoords.header = pta->header;
            poseInCamCoords.pose.position.x = pt.traj_x_camera[0];
            poseInCamCoords.pose.position.y = pt.traj_y_camera[0];
            poseInCamCoords.pose.position.z = pt.traj_z_camera[0];

            //Counteracting rotation in tf because it is already done in the pedetrsian tracking.
            //tf rotation 0.5 -0.5 0.5 0.5 or 1.571, -1.571 0.0
            //tf::Quaternion rot;
            //rot.setRPY(-1.571, 1.571, 0.0);
            poseInCamCoords.pose.orientation.x = -0.5; //rot.getX()
            poseInCamCoords.pose.orientation.y =  0.5; //rot.getY()
            poseInCamCoords.pose.orientation.z =  0.5; //rot.getZ()
            poseInCamCoords.pose.orientation.w =  0.5; //rot.getW()

            //Transform
            try {
                listener->transformPose("/base_link", poseInCamCoords, poseInRobotCoords);
            }
            catch(tf::TransformException ex) {
                ROS_ERROR("Failed transform: %s", ex.what());
            }
            ROS_DEBUG("pta: Position robot x: %f, y: %f, z: %f",
                     poseInRobotCoords.pose.position.x,
                     poseInRobotCoords.pose.position.y,
                     poseInRobotCoords.pose.position.z);
        }
    }

}

void ubdCallback(const UpperBodyDetector::ConstPtr &msg)
{
    for(int i = 0; i < msg->pos_x.size(); i++) {
        if(msg->pos_x.size()>i && msg->pos_y.size()>i && msg->dist.size()>i) {
            ROS_DEBUG("ubd: Position x: %i, y: %i, z: %i", (int)msg->pos_x[i], (int)msg->pos_y[i], (int)msg->dist[i]);
        }
    }
    //TODO: Fill with magic
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "pedestrian_localisation");
    ros::NodeHandle n;

    listener = new tf::TransformListener();

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


