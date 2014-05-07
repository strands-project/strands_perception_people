#include <ros/ros.h>

#include "strands_perception_people_msgs/StartHeadAnalysis.h"
using namespace strands_perception_people_msgs;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_head_analysis");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<StartHeadAnalysis>("start_head_analysis");

    StartHeadAnalysis msg;
    while(! client.call(msg)) {
        ROS_ERROR("Failed to start head analysis. Never gonna give you up!");
        if(! ros::Duration(2.0).sleep())
            return 1;
    }

    if(msg.response.was_running) {
        ROS_INFO("Head analysis was already started.");
    } else {
        ROS_INFO("Head analysis started.");
    }

    return 0;
}

