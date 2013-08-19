#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <string.h>
#include <vector>
#include <math.h>

#include "strands_perception_people_msgs/PedestrianTracking.h"
#include "strands_perception_people_msgs/PedestrianTrackingArray.h"
#include "strands_perception_people_msgs/UpperBodyDetector.h"
#include "strands_perception_people_msgs/PedestrianLocations.h"

using namespace std;
using namespace strands_perception_people_msgs;

ros::Publisher pub_detect;
ros::Publisher pub_marker;
tf::TransformListener* listener;

string target_frame;
int detect_seq = 0;
int marker_seq = 0;

void publishDetections(vector<geometry_msgs::Point> ppl, vector<double> distances, double min_dist) {
    PedestrianLocations result;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = target_frame;
    result.header.seq = ++detect_seq;
    for(int i = 0; i < ppl.size(); i++) {
        geometry_msgs::Pose pose;
        pose.position.x = ppl[i].x;
        pose.position.y = ppl[i].y;
        pose.position.z = ppl[i].z;
        //TODO: Get orientation from direction estimation
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        result.poses.push_back(pose);
        ROS_DEBUG("publishDetections: Publishing detection: x: %f, y: %f, z: %f",
                 pose.position.x,
                 pose.position.y,
                 pose.position.z);
    }
    result.distances = distances;
    result.min_distance = min_dist;
    pub_detect.publish(result);
}

void createVisualisation(std::vector<geometry_msgs::Point> points) {
    ROS_DEBUG("Creating markers");
    visualization_msgs::MarkerArray marker_array;
    for(int i = 0; i < points.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = target_frame;
        marker.header.stamp = ros::Time::now();
        marker.header.seq = ++marker_seq;
        marker.ns = "pedestrian_localisation";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = points[i].x;
        marker.pose.position.y = points[i].y;
        marker.pose.position.z = 0.6;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 1.2;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
    }
    pub_marker.publish(marker_array);
}

vector<float> polarToCartesian(geometry_msgs::Point point) {
    ROS_DEBUG("cartesianToPolar: Cartesian point: x: %f, y: %f, z %f", point.x, point.y, point.z);
    vector<float> output;
    float dist = sqrt(pow(point.x,2) + pow(point.y,2));
    float angle = atan2(point.y, point.x);
    output.push_back(dist);
    output.push_back(angle);
    ROS_DEBUG("cartesianToPolar: Polar point: distance: %f, angle: %f", dist, angle);
    return output;
}

void trackingCallback(const PedestrianTrackingArray::ConstPtr &pta)
{
    bool loc = pub_detect.getNumSubscribers();
    bool markers = pub_marker.getNumSubscribers();
    bool dist;
    if(!loc && !markers) {
        ROS_DEBUG("No subscribers. Skipping calculation.");
        return;
    }


    vector<geometry_msgs::Point> ppl;
    vector<double> distances;
    double min_dist = 10000.0d;
    for(int i = 0; i < pta->pedestrians.size(); i++) {
        PedestrianTracking pt = pta->pedestrians[i];
        if(pt.traj_x.size() && pt.traj_y.size() && pt.traj_z.size()) {
            ROS_DEBUG("trackingCallback: Received: Position world x: %f, y: %f, z: %f",
                     pt.traj_x[0],
                     pt.traj_y[0],
                     pt.traj_z[0]);
            ROS_DEBUG("trackingCallback: Received: Position cam x: %f, y: %f, z: %f",
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
                ROS_DEBUG("Transforming received position into %s coordinate system.", target_frame.c_str());
                listener->waitForTransform(poseInCamCoords.header.frame_id, target_frame, ros::Time(), ros::Duration(3.0));
                listener->transformPose(target_frame, poseInCamCoords, poseInRobotCoords);
            }
            catch(tf::TransformException ex) {
                ROS_ERROR("Failed transform: %s", ex.what());
            }
            ppl.push_back(poseInRobotCoords.pose.position);
            double dist = (double)polarToCartesian(poseInRobotCoords.pose.position)[0];
            distances.push_back(dist);
            min_dist = dist < min_dist ? dist : min_dist;
        }
        publishDetections(ppl, distances, min_dist);
        if(markers)
            createVisualisation(ppl);
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
    string pub_marker_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("target_frame", target_frame, string("/base_link"));
    private_node_handle_.param("pedestrian_array", pta_topic, string("/pedestrian_tracking/pedestrian_array"));
    private_node_handle_.param("upper_body_detections", ubd_topic, string("/upper_body_detector/detections"));

    // Create a subscriber.
    ros::Subscriber pta_sub = n.subscribe(pta_topic.c_str(), 10, &trackingCallback);
//    ros::Subscriber ubd_sub = n.subscribe(ubd_topic.c_str(), 10, &ubdCallback);

    private_node_handle_.param("localisations", pub_topic, string("/pedestrian_localisation/localisations"));
    pub_detect = n.advertise<PedestrianLocations>(pub_topic.c_str(), 10);
    private_node_handle_.param("marker", pub_marker_topic, string("/pedestrian_localisation/marker_array"));
    pub_marker = n.advertise<visualization_msgs::MarkerArray>(pub_marker_topic.c_str(), 10);

    ros::spin();
    return 0;
}


