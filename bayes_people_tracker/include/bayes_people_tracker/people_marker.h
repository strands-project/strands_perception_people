#ifndef PEOPLE_MARKER_H
#define PEOPLE_MARKER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>


#include <string.h>
#include <vector>

class PeopleMarker {

public:
    PeopleMarker() : marker_seq(0) {}

    geometry_msgs::Point generate_position(geometry_msgs::Point centre, double angle, double dx, double dy)
    {
        float s = sin(angle);
        float c = cos(angle);

        // rotate point
        geometry_msgs::Point res;
        res.x = dx * c - dy * s;
        res.y = dx * s + dy * c;

        // translate point back:
        res.x += centre.x;
        res.y += centre.y;
        res.z  = centre.z;
        return res;
    }

    geometry_msgs::Pose generate_extremity_position(geometry_msgs::Pose centre, double dx, double dy, double z) {
        double angle = tf::getYaw(centre.orientation) + M_PI/2;
        geometry_msgs::Point p = centre.position;
        p.z = z;
        centre.position = generate_position(p, angle, dx, dy);
        return centre;
    }

    visualization_msgs::Marker createMarker(
            int id,
            int type,
            int action,
            geometry_msgs::Pose pose,
            geometry_msgs::Vector3 scale,
            std_msgs::ColorRGBA color,
            std::string target_frame) {
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
            int action,
            geometry_msgs::Pose pose,
            std::string target_frame) {
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
        return createMarker(id, visualization_msgs::Marker::SPHERE, action, pose, scale, color, target_frame);
    }

    visualization_msgs::Marker createBody(
            int id,
            int action,
            geometry_msgs::Pose pose,
            std::string target_frame) {
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
        return createMarker(id, visualization_msgs::Marker::CYLINDER, action, pose, scale, color, target_frame);
    }

    std::vector<visualization_msgs::Marker> createLegs(
            int idl, int idr,
            int action,
            geometry_msgs::Pose pose,
            std::string target_frame) {
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
        legs.push_back(createMarker(idl, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, 0.1, 0.0, 0.4), scale, color, target_frame));
        legs.push_back(createMarker(idr, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, -0.1, 0.0, 0.4), scale, color, target_frame));
        return legs;
    }

    std::vector<visualization_msgs::Marker> createArms(
            int idl, int idr,
            int action,
            geometry_msgs::Pose pose,
            std::string target_frame) {
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
        arms.push_back(createMarker(idl, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, 0.2, 0.0, 1.1), scale, color, target_frame));
        arms.push_back(createMarker(idr, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, -0.2, 0.0, 1.1), scale, color, target_frame));
        return arms;
    }

    std::vector<visualization_msgs::Marker> createHuman(
            int id,
            geometry_msgs::Pose pose,
            std::string target_frame) {
        std::vector<visualization_msgs::Marker> human;
        human.push_back(createHead(id++, visualization_msgs::Marker::ADD, pose, target_frame));
        human.push_back(createBody(id++, visualization_msgs::Marker::ADD, pose, target_frame));
        std::vector<visualization_msgs::Marker> legs = createLegs(id++, id++, visualization_msgs::Marker::ADD, pose, target_frame);
        human.insert(human.end(), legs.begin(), legs.end());
        std::vector<visualization_msgs::Marker> arms = createArms(id++, id++, visualization_msgs::Marker::ADD, pose, target_frame);
        human.insert(human.end(), arms.begin(), arms.end());
        return human;
    }

private:

    unsigned long marker_seq;

};


#endif // PEOPLE_MARKER_H
