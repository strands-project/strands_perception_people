#include <ros/ros.h>

#include "strands_perception_people_msgs/IsHeadAnalysisRunning.h"
using namespace strands_perception_people_msgs;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "status_head_analysis");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<IsHeadAnalysisRunning>("status_head_analysis");

    IsHeadAnalysisRunning msg;
    if(! client.call(msg)) {
        ROS_ERROR("Failed to check head analysis.");
        return 1;
    }

    std::cout << (msg.response.is_running ? "running" : "stopped") << std::endl;
    return 0;
}

