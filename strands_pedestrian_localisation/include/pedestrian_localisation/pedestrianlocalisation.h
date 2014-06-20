#ifndef PEDESTRIANLOCALISATION_H
#define PEDESTRIANLOCALISATION_H

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

class PedestrianLocalisation
{
public:
    PedestrianLocalisation();

private:
    void publishDetections(std::vector<geometry_msgs::Point> ppl, std::vector<int> ids, std::vector<double> distances, std::vector<double> angles, double min_dist, double angle);
    void createVisualisation(std::vector<geometry_msgs::Point> points);
    std::vector<double> cartesianToPolar(geometry_msgs::Point point);
    void trackingCallback(const strands_perception_people_msgs::PedestrianTrackingArray::ConstPtr &pta);
    void connectCallback(ros::NodeHandle &n, ros::Subscriber &sub, std::string topic);

    ros::Publisher pub_detect;
    ros::Publisher pub_pose;
    ros::Publisher pub_marker;
    tf::TransformListener* listener;
    std::string target_frame;
    unsigned long detect_seq;
    unsigned long marker_seq;
};

#endif // PEDESTRIANLOCALISATION_H
