#ifndef PEDESTRIANLOCALISATION_H
#define PEDESTRIANLOCALISATION_H

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <bayes_tracking/BayesFilter/bayesFlt.hpp>

#include <XmlRpcValue.h>

#include <string.h>
#include <vector>
#include <math.h>

#include "bayes_people_tracker/PeopleTracker.h"
#include "bayes_people_tracker/simple_tracking.h"
#include "bayes_people_tracker/asso_exception.h"
#include "bayes_people_tracker/people_marker.h"

#define BASE_LINK "/base_link"

class PeopleTracker
{
public:
    PeopleTracker();

private:
    void trackingThread();
    void publishDetections(bayes_people_tracker::PeopleTracker msg);
    void publishDetections(geometry_msgs::PoseStamped msg);
    void publishDetections(geometry_msgs::PoseArray msg);
    void publishDetections(people_msgs::People msg);
    void publishDetections(double time_sec,
                           geometry_msgs::Pose closest,
                           std::vector<geometry_msgs::Pose> ppl,
                           std::vector<geometry_msgs::Pose> vels,
                           std::vector<std::string> uuids,
                           std::vector<double> distances,
                           std::vector<double> angles,
                           double min_dist,
                           double angle);
    void createVisualisation(std::vector<geometry_msgs::Pose> points, ros::Publisher& pub);
    std::vector<double> cartesianToPolar(geometry_msgs::Point point);
    void detectorCallback(const geometry_msgs::PoseArray::ConstPtr &pta, string detector);
    void connectCallback(ros::NodeHandle &n);
    void parseParams(ros::NodeHandle);

    std::string generateUUID(std::string time, long id) {
        boost::uuids::name_generator gen(dns_namespace_uuid);
        time += num_to_str<long>(id);

        return num_to_str<boost::uuids::uuid>(gen(time.c_str()));
    }

    template<typename T>
    std::string num_to_str(T num) {
        std::stringstream ss;
        ss << num;
        return ss.str();
    }

    ros::Publisher pub_detect;
    ros::Publisher pub_pose;
    ros::Publisher pub_pose_array;
    ros::Publisher pub_people;
    ros::Publisher pub_marker;
    tf::TransformListener* listener;
    std::string target_frame;
    unsigned long detect_seq;
    double startup_time;
    std::string startup_time_str;

    boost::uuids::uuid dns_namespace_uuid;

    PeopleMarker pm;

    SimpleTracking<EKFilter> *ekf = NULL;
    SimpleTracking<UKFilter> *ukf = NULL;
    std::map<std::pair<std::string, std::string>, ros::Subscriber> subscribers;
};

#endif // PEDESTRIANLOCALISATION_H
