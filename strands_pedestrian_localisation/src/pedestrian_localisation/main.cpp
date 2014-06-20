#include "pedestrian_localisation/pedestrianlocalisation.h"


int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "pedestrian_localisation");
    PedestrianLocalisation* pl = new PedestrianLocalisation();
    return 0;
}


