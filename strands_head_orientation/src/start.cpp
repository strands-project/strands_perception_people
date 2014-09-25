#include <ros/ros.h>

#include "strands_head_orientation/StartHeadAnalysis.h"
using namespace strands_head_orientation;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_head_analysis");

    std::string name = argc > 1 && std::string(argv[1]) == "recording"
        ? "recording"
        : "analysis";

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<StartHeadAnalysis>("start_head_" + name);

    StartHeadAnalysis msg;
    while(! client.call(msg)) {
        ROS_ERROR("Failed to start head %s. Never gonna give you up!", name.c_str());
        if(! ros::Duration(2.0).sleep())
            return 1;
    }

    if(msg.response.was_running) {
        ROS_INFO("Head %s was already started.", name.c_str());
    } else {
        ROS_INFO("Head %s started.", name.c_str());
    }

    return 0;
}

