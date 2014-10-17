#ifndef VISUALISATIONMARKERS_H
#define VISUALISATIONMARKERS_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>

#include <string.h>
#include <vector>

class VisualisationMarkers {
public:
    VisualisationMarkers():
        marker_seq(0) {}

    visualization_msgs::Marker createMarker(
            int id,
            std::string target_frame,
            int type,
            int action,
            geometry_msgs::Pose pose,
            geometry_msgs::Vector3 scale,
            std_msgs::ColorRGBA color) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = target_frame;
        marker.header.stamp = ros::Time::now();
        marker.header.seq = ++marker_seq;
        marker.ns = "people_tracker";
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
            std::string target_frame,
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
        return createMarker(id, target_frame, visualization_msgs::Marker::SPHERE, action, pose, scale, color);
    }

    visualization_msgs::Marker createBody(
            int id,
            std::string target_frame,
            int action,
            geometry_msgs::Pose pose) {
        geometry_msgs::Vector3 scale;
        scale.x = 0.35;
        scale.y = 0.35;
        scale.z = 0.7;
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.r = 0.0F/255.0F;
        color.g = 140.0F/255.0F;
        color.b = 0.0F/255.0F;
        pose.position.z = 1.1;
        return createMarker(id, target_frame, visualization_msgs::Marker::CYLINDER, action, pose, scale, color);
    }

    std::vector<visualization_msgs::Marker> createLegs(
            int idl, int idr,
            std::string target_frame,
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
        legs.push_back(createMarker(idl, target_frame, visualization_msgs::Marker::CYLINDER, action, pose, scale, color));
        pose.position.x -= 0.2;
        legs.push_back(createMarker(idr, target_frame, visualization_msgs::Marker::CYLINDER, action, pose, scale, color));
        return legs;
    }

    std::vector<visualization_msgs::Marker> createArms(
            int idl, int idr,
            std::string target_frame,
            int action,
            geometry_msgs::Pose pose) {
        std::vector<visualization_msgs::Marker> arms;
        geometry_msgs::Vector3 scale;
        scale.x = 0.1;
        scale.y = 0.1;
        scale.z = 0.7;
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.r = 0.0F/255.0F;
        color.g = 140.0F/255.0F;
        color.b = 0.0F/255.0F;
        pose.position.z = 1.1;
        pose.position.x += 0.2;
        arms.push_back(createMarker(idl, target_frame, visualization_msgs::Marker::CYLINDER, action, pose, scale, color));
        pose.position.x -= 0.4;
        arms.push_back(createMarker(idr, target_frame, visualization_msgs::Marker::CYLINDER, action, pose, scale, color));
        return arms;
    }

    std::vector<visualization_msgs::Marker> createHuman(
            int id,
            std::string target_frame,
            geometry_msgs::Pose pose) {
        std::vector<visualization_msgs::Marker> human;
        human.push_back(createHead(id++, target_frame, visualization_msgs::Marker::ADD, pose));
        human.push_back(createBody(id++, target_frame, visualization_msgs::Marker::ADD, pose));
        std::vector<visualization_msgs::Marker> legs = createLegs(id++, id++, target_frame, visualization_msgs::Marker::ADD, pose);
        human.insert(human.end(), legs.begin(), legs.end());
        std::vector<visualization_msgs::Marker> arms = createArms(id++, id++, target_frame, visualization_msgs::Marker::ADD, pose);
        human.insert(human.end(), arms.begin(), arms.end());
        return human;
    }

private:
    unsigned long marker_seq;
};



#endif // VISUALISATIONMARKERS_H
