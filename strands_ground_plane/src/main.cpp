// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>

#include "strands_perception_people_msgs/GroundPlane.h"

using namespace std;
using namespace strands_perception_people_msgs;

ros::Publisher pub_ground_plane;


void callback(const sensor_msgs::JointState::ConstPtr &msg) {
    double tilt = 0.0;
    for(int i = 0; i < msg->name.size(); i++){
        if(strcmp(msg->name[i].c_str(),"tilt"))
            tilt = msg->position[i];
    }

    //Magic numbers!
    tf::Vector3 n(0.012713099792961273, -0.999862809242558, 0.010617899379562451);
    double d = 1.94430405427;

    //Rotate
    n.rotate(tf::Vector3(1,0,0),tilt);
    GroundPlane gp;
    gp.header = msg->header;
    gp.n.push_back(n.getX());
    gp.n.push_back(n.getY());
    gp.n.push_back(n.getZ());
    gp.d = d;
    pub_ground_plane.publish(gp);
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "ground_plane");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    string pub_topic_gp;
    string sub_ptu_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("ptu_state", sub_ptu_topic, string("/ptu/state"));

    // Create a subscriber.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    ros::Subscriber ptu_sub = n.subscribe(sub_ptu_topic.c_str(), 10, &callback);

    // Create a topic publisher
    private_node_handle_.param("ground_plane", pub_topic_gp, string("/ground_plane"));
    pub_ground_plane = n.advertise<GroundPlane>(pub_topic_gp.c_str(), 10);


    ros::spin();

    return 0;
}

