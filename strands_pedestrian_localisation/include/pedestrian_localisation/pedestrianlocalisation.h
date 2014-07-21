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
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <string.h>
#include <vector>
#include <math.h>

#include "strands_perception_people_msgs/PedestrianTracking.h"
#include "strands_perception_people_msgs/PedestrianTrackingArray.h"
#include "strands_perception_people_msgs/UpperBodyDetector.h"
#include "strands_perception_people_msgs/PedestrianLocations.h"

#define BASE_LINK "/base_link"

class PedestrianLocalisation
{
public:
    PedestrianLocalisation();

private:
    void publishDetections(std_msgs::Header header,
                           std::vector<geometry_msgs::Point> ppl,
                           std::vector<int> ids,
                           std::vector<std::string> uuids,
                           std::vector<double> scores,
                           std::vector<double> distances,
                           std::vector<double> angles,
                           double min_dist,
                           double angle);
    void createVisualisation(std::vector<geometry_msgs::Point> points);
    std::vector<double> cartesianToPolar(geometry_msgs::Point point);
    void trackingCallback(const strands_perception_people_msgs::PedestrianTrackingArray::ConstPtr &pta);
    void connectCallback(ros::NodeHandle &n, ros::Subscriber &sub, std::string topic);

    visualization_msgs::Marker createMarker(
            int id,
            int type,
            int action,
            geometry_msgs::Pose pose,
            geometry_msgs::Vector3 scale,
            std_msgs::ColorRGBA color) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = target_frame;
        marker.header.stamp = ros::Time::now();
        marker.header.seq = ++marker_seq;
        marker.ns = "pedestrian_localisation";
        marker.id = id;
        marker.type = type;
        marker.action = action;
        marker.pose = pose;
        marker.scale = scale;
        marker.color = color;
        marker.lifetime = ros::Duration(1);
        return marker;
    }

    visualization_msgs::Marker createHead(
            int id,
            int action,
            geometry_msgs::Pose pose) {
        geometry_msgs::Vector3 scale;
        scale.x = 0.3;
        scale.y = 0.3;
        scale.z = 0.3;
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.r = 233.0F/255.0F;
        color.g = 150.0F/255.0F;
        color.b = 122.0F/255.0F;
        pose.position.z = 1.6;
        return createMarker(id, visualization_msgs::Marker::SPHERE, action, pose, scale, color);
    }

    visualization_msgs::Marker createBody(
            int id,
            int action,
            geometry_msgs::Pose pose) {
        geometry_msgs::Vector3 scale;
        scale.x = 0.35;
        scale.y = 0.35;
        scale.z = 0.7;
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.r = 139.0F/255.0F;
        color.g = 0.0F/255.0F;
        color.b = 0.0F/255.0F;
        pose.position.z = 1.1;
        return createMarker(id, visualization_msgs::Marker::CYLINDER, action, pose, scale, color);
    }

    std::vector<visualization_msgs::Marker> createLegs(
            int idl, int idr,
            int action,
            geometry_msgs::Pose pose) {
        std::vector<visualization_msgs::Marker> legs;
        geometry_msgs::Vector3 scale;
        scale.x = 0.15;
        scale.y = 0.2;
        scale.z = 0.8;
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.r = 0.0F/255.0F;
        color.g = 0.0F/255.0F;
        color.b = 139.0F/255.0F;
        pose.position.z = 0.4;
        pose.position.x += 0.1;
        legs.push_back(createMarker(idl, visualization_msgs::Marker::CYLINDER, action, pose, scale, color));
        pose.position.x -= 0.2;
        legs.push_back(createMarker(idr, visualization_msgs::Marker::CYLINDER, action, pose, scale, color));
        return legs;
    }

    std::vector<visualization_msgs::Marker> createArms(
            int idl, int idr,
            int action,
            geometry_msgs::Pose pose) {
        std::vector<visualization_msgs::Marker> arms;
        geometry_msgs::Vector3 scale;
        scale.x = 0.1;
        scale.y = 0.1;
        scale.z = 0.7;
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.r = 139.0F/255.0F;
        color.g = 0.0F/255.0F;
        color.b = 0.0F/255.0F;
        pose.position.z = 1.1;
        pose.position.x += 0.2;
        arms.push_back(createMarker(idl, visualization_msgs::Marker::CYLINDER, action, pose, scale, color));
        pose.position.x -= 0.4;
        arms.push_back(createMarker(idr, visualization_msgs::Marker::CYLINDER, action, pose, scale, color));
        return arms;
    }

    std::vector<visualization_msgs::Marker> createHuman(
            int id,
            geometry_msgs::Pose pose) {
        std::vector<visualization_msgs::Marker> human;
        human.push_back(createHead(id++, visualization_msgs::Marker::ADD, pose));
        human.push_back(createBody(id++, visualization_msgs::Marker::ADD, pose));
        std::vector<visualization_msgs::Marker> legs = createLegs(id++, id++, visualization_msgs::Marker::ADD, pose);
        human.insert(human.end(), legs.begin(), legs.end());
        std::vector<visualization_msgs::Marker> arms = createArms(id++, id++, visualization_msgs::Marker::ADD, pose);
        human.insert(human.end(), arms.begin(), arms.end());
        return human;
    }

    ros::Publisher pub_detect;
    ros::Publisher pub_pose;
    ros::Publisher pub_marker;
    tf::TransformListener* listener;
    std::string target_frame;
    unsigned long detect_seq;
    unsigned long marker_seq;
};

#endif // PEDESTRIANLOCALISATION_H
