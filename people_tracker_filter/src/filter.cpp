#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <geometry_msgs/Point.h>
#include <bayes_people_tracker/PeopleTracker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher pub, mpub;
nav_msgs::OccupancyGrid map;

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map = *msg;
}

void posistion_callback(const bayes_people_tracker::PeopleTracker::ConstPtr& msg) {
    if(msg->poses.size() != 0){
        bayes_people_tracker::PeopleTracker out;
        out.header = msg->header;
        for(int i = 0; i < msg->poses.size(); i++){
            try {
                if(int(map.data.at(occupancy_grid_utils::pointIndex(map.info, msg->poses[i].position))) == 0) {
                    out.distances.push_back(msg->distances[i]);
                    out.angles.push_back(msg->angles[i]);
                    out.poses.push_back(msg->poses[i]);
                    out.uuids.push_back(msg->uuids[i]);
                    out.velocities.push_back(msg->velocities[i]);
                }
            } catch (occupancy_grid_utils::CellOutOfBoundsException) {
                ROS_WARN("Cell out of bounds");
                continue;
            }
        }
        if(out.distances.size() != 0){
            out.min_distance = *std::min_element(out.distances.begin(), out.distances.end());
            out.min_distance_angle = out.angles[std::distance(out.distances.begin(), std::min_element(out.distances.begin(), out.distances.end()))];
        }
        pub.publish(out);
    } else {
        pub.publish(msg);
    }
}

void marker_callback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    if(msg->markers.size() != 0){
        visualization_msgs::MarkerArray out;
        for(int i = 0; i < msg->markers.size(); i++){
            try {
                if(int(map.data.at(occupancy_grid_utils::pointIndex(map.info, msg->markers[i].pose.position))) == 0) {
                    out.markers.push_back(msg->markers[i]);
                }
            } catch (occupancy_grid_utils::CellOutOfBoundsException) {
                ROS_WARN("Cell out of bounds");
                continue;
            }
        }
        mpub.publish(out);
    } else {
        mpub.publish(msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "people_tracker_filter");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/ppl_map", 1, map_callback);
    while(map.header.stamp.sec == 0 and ros::ok()){
        ROS_INFO("Waiting for map");
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
    pub = n.advertise<bayes_people_tracker::PeopleTracker>("/people_tracker_filter/positions", 10);
    mpub = n.advertise<visualization_msgs::MarkerArray>("/people_tracker_filter/marker_array", 10);
    ros::Subscriber ppl_sub = n.subscribe("/people_tracker/positions", 1, posistion_callback);
    ros::Subscriber marker_sub = n.subscribe("/people_tracker/marker_array", 1, marker_callback);

//    ROS_INFO_STREAM("" << map.info);
//    occupancy_grid_utils::Cell c(0,0);
//    geometry_msgs::Point p;
//    p.x = 0;
//    p.y = 0;
//    ROS_INFO_STREAM("" << int(map.data.at(occupancy_grid_utils::pointIndex(map.info, p))));

    ros::spin();

    return 0;
}
