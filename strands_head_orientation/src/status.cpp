#include <ros/ros.h>

#include "strands_head_orientation/IsHeadAnalysisRunning.h"
using namespace strands_head_orientation;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "status_head_analysis");

    std::string name = argc > 1 && std::string(argv[1]) == "recording"
        ? "recording"
        : "analysis";

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<IsHeadAnalysisRunning>("status_head_" + name);

    IsHeadAnalysisRunning msg;
    if(! client.call(msg)) {
        ROS_ERROR("Failed to check head %s.", name.c_str());
        return 1;
    }

    std::cout << (msg.response.is_running ? "running" : "stopped") << std::endl;
    return 0;
}

