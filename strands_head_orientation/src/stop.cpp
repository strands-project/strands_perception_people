#include <ros/ros.h>

#include "strands_perception_people_msgs/StopHeadAnalysis.h"
using namespace strands_perception_people_msgs;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stop_head_analysis");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<StopHeadAnalysis>("stop_head_analysis");

    StopHeadAnalysis msg;
    while(! client.call(msg)) {
        ROS_ERROR("Failed to start head analysis. Never gonna give you up!");
        if(! ros::Duration(2.0).sleep())
            return 1;
    }

    if(msg.response.was_running) {
        ROS_INFO("Head analysis stopped.");
    } else {
        ROS_INFO("Head analysis wasn't even started.");
    }

    return 0;
}

