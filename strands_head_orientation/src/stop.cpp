#include <ros/ros.h>

#include "strands_head_orientation/StopHeadAnalysis.h"
using namespace strands_head_orientation;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stop_head_analysis");

    std::string name = argc > 1 && std::string(argv[1]) == "recording"
        ? "recording"
        : "analysis";

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<StopHeadAnalysis>("stop_head_" + name);

    StopHeadAnalysis msg;
    while(! client.call(msg)) {
        ROS_ERROR("Failed to start head %s. Never gonna give you up!", name.c_str());
        if(! ros::Duration(2.0).sleep())
            return 1;
    }

    if(msg.response.was_running) {
        ROS_INFO("Head %s stopped.", name.c_str());
    } else {
        ROS_INFO("Head %s wasn't even started.", name.c_str());
    }

    return 0;
}

