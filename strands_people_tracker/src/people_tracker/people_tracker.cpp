#include "people_tracker/people_tracker.h"

PeopleTracker::PeopleTracker() :
    detect_seq(0),
    marker_seq(0)
{
    ros::NodeHandle n;

    listener = new tf::TransformListener();

    // Declare variables that can be modified by launch file or command line.
    std::string pta_topic;
    std::string pub_topic;
    std::string pub_topic_pose;
    std::string pub_marker_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("target_frame", target_frame, std::string("/base_link"));
//    private_node_handle_.param("pedestrian_array", pta_topic, std::string("/pedestrian_tracking/pedestrian_array"));
    private_node_handle_.param("pedestrian_array", pta_topic, std::string("/upper_body_detector/bounding_box_centres"));

    // Create a subscriber.
    ros::Subscriber pta_sub;
    ros::SubscriberStatusCallback con_cb = boost::bind(&PeopleTracker::connectCallback, this, boost::ref(n), boost::ref(pta_sub), pta_topic);

    private_node_handle_.param("positions", pub_topic, std::string("/pedestrian_localisation/localisations"));
    pub_detect = n.advertise<strands_perception_people_msgs::PeopleTracker>(pub_topic.c_str(), 10, con_cb, con_cb);
    private_node_handle_.param("pose", pub_topic_pose, std::string("/pedestrian_localisation/pose"));
    pub_pose = n.advertise<geometry_msgs::PoseStamped>(pub_topic_pose.c_str(), 10, con_cb, con_cb);
    private_node_handle_.param("marker", pub_marker_topic, std::string("/pedestrian_localisation/marker_array"));
    pub_marker = n.advertise<visualization_msgs::MarkerArray>(pub_marker_topic.c_str(), 10, con_cb, con_cb);
    test_marker = n.advertise<visualization_msgs::MarkerArray>("/test/markers", 10, con_cb, con_cb);

    st = new SimpleTracking();
    boost::thread tracking_thread(boost::bind(&PeopleTracker::trackingThread, this));

    ros::spin();
}

void PeopleTracker::trackingThread() {
    ros::Rate fps(25);
    while(ros::ok()) {
        std::vector<geometry_msgs::Point> ppl = st->track();
        if(pub_marker.getNumSubscribers())
            createVisualisation(ppl, pub_marker);
        fps.sleep();
    }
}

void PeopleTracker::publishDetections(
        std_msgs::Header header,
        std::vector<geometry_msgs::Point> ppl,
        std::vector<int> ids,
        std::vector<std::string> uuids,
        std::vector<double> scores,
        std::vector<double> distances,
        std::vector<double> angles,
        double min_dist,
        double angle) {
    strands_perception_people_msgs::PeopleTracker result;
    result.header.stamp = header.stamp;
    result.header.frame_id = target_frame;
    result.header.seq = ++detect_seq;
    for(int i = 0; i < ppl.size(); i++) {
        geometry_msgs::Pose pose;
        pose.position.x = ppl[i].x;
        pose.position.y = ppl[i].y;
        pose.position.z = ppl[i].z;
        //TODO: Get orientation from direction estimation
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        result.poses.push_back(pose);
        ROS_DEBUG("publishDetections: Publishing detection: x: %f, y: %f, z: %f",
                  pose.position.x,
                  pose.position.y,
                  pose.position.z);
    }
    result.ids = ids;
    result.uuids = uuids;
    result.scores = scores;
    result.distances = distances;
    result.angles = angles;
    result.min_distance = min_dist;
    result.min_distance_angle = angle;
    publishDetections(result);
}

void PeopleTracker::publishDetections(strands_perception_people_msgs::PeopleTracker msg) {
    pub_detect.publish(msg);
}

void PeopleTracker::createVisualisation(std::vector<geometry_msgs::Point> points, ros::Publisher& pub) {
    ROS_DEBUG("Creating markers");
    visualization_msgs::MarkerArray marker_array;
    for(int i = 0; i < points.size(); i++) {

        geometry_msgs::Pose pose;
        pose.position.x = points[i].x;
        pose.position.y = points[i].y;
        pose.position.z = 0.6;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        std::vector<visualization_msgs::Marker> human = createHuman(i*10, pose);

        marker_array.markers.insert(marker_array.markers.begin(), human.begin(), human.end());
    }
    pub.publish(marker_array);
}

std::vector<double> PeopleTracker::cartesianToPolar(geometry_msgs::Point point) {
    ROS_DEBUG("cartesianToPolar: Cartesian point: x: %f, y: %f, z %f", point.x, point.y, point.z);
    std::vector<double> output;
    double dist = sqrt(pow(point.x,2) + pow(point.y,2));
    double angle = atan2(point.y, point.x);
    output.push_back(dist);
    output.push_back(angle);
    ROS_DEBUG("cartesianToPolar: Polar point: distance: %f, angle: %f", dist, angle);
    return output;
}

//void PedestrianLocalisation::trackingCallback(const strands_perception_people_msgs::PedestrianTrackingArray::ConstPtr &pta)
void PeopleTracker::detectorCallback(const geometry_msgs::PoseArray::ConstPtr &pta)
{
    // Publish an empty message to trigger callbacks even when there are no detections.
    // This can be used by nodes which might also want to know when there is no human detected.
//    if(pta->pedestrians.size() == 0) {
    if(pta->poses.size() == 0) {
        strands_perception_people_msgs::PeopleTracker empty;
        empty.header.stamp = ros::Time::now();
        empty.header.frame_id = target_frame;
        empty.header.seq = ++detect_seq;
        publishDetections(empty);
        return;
    }

    geometry_msgs::PoseStamped closest_person;
    std::vector<geometry_msgs::Point> ppl;
    std::vector<int> ids;
    std::vector<std::string> uuids;
    std::vector<double> scores;
    std::vector<double> distances;
    std::vector<double> angles;
    double min_dist = 10000.0d;
    double angle;
//    for(int i = 0; i < pta->pedestrians.size(); i++) {
    for(int i = 0; i < pta->poses.size(); i++) {
//        strands_perception_people_msgs::PedestrianTracking pt = pta->pedestrians[i];
        geometry_msgs::Pose pt = pta->poses[i];
        if(/*pt.traj_x.size() && pt.traj_y.size() && pt.traj_z.size()*/ 1) {
//            ROS_DEBUG("trackingCallback: Received: Position world x: %f, y: %f, z: %f",
//                      pt.traj_x[0],
//                      pt.traj_y[0],
//                      pt.traj_z[0]);
//            ROS_DEBUG("trackingCallback: Received: Position cam x: %f, y: %f, z: %f",
//                      pt.traj_x_camera[0],
//                      pt.traj_y_camera[0],
//                      pt.traj_z_camera[0]);

            //Create stamped pose for tf
            geometry_msgs::PoseStamped poseInCamCoords;
            geometry_msgs::PoseStamped poseInRobotCoords;
            geometry_msgs::PoseStamped poseInTargetCoords;
            poseInCamCoords.header = pta->header;
//            poseInCamCoords.pose.position.x = pt.traj_x_camera[0];
//            poseInCamCoords.pose.position.y = pt.traj_y_camera[0];
//            poseInCamCoords.pose.position.z = pt.traj_z_camera[0];
            poseInCamCoords.pose = pt;

//            //Counteracting rotation in tf because it is already done in the pedestrian tracking.
//            poseInCamCoords.pose.orientation.x = -0.5;
//            poseInCamCoords.pose.orientation.y =  0.5;
//            poseInCamCoords.pose.orientation.z =  0.5;
//            poseInCamCoords.pose.orientation.w =  0.5;

            //Transform
            try {
                // Transform into robot coordinate system /base_link for the caluclation of relative distances and angles
                ROS_DEBUG("Transforming received position into %s coordinate system.", BASE_LINK);
                listener->waitForTransform(poseInCamCoords.header.frame_id, BASE_LINK, poseInCamCoords.header.stamp, ros::Duration(3.0));
                listener->transformPose(BASE_LINK, ros::Time(0), poseInCamCoords, poseInCamCoords.header.frame_id, poseInRobotCoords);

                // Transform into given traget frame. Default /map
                if(strcmp(target_frame.c_str(), BASE_LINK)) {
                    ROS_DEBUG("Transforming received position into %s coordinate system.", target_frame.c_str());
                    listener->waitForTransform(poseInCamCoords.header.frame_id, target_frame, poseInCamCoords.header.stamp, ros::Duration(3.0));
                    listener->transformPose(target_frame, ros::Time(0), poseInCamCoords, poseInCamCoords.header.frame_id, poseInTargetCoords);
                } else {
                    poseInTargetCoords = poseInRobotCoords;
                }
            }
            catch(tf::TransformException ex) {
                ROS_WARN("Failed transform: %s", ex.what());
                return;
            }

            poseInTargetCoords.pose.position.z = 0.0;
            poseInRobotCoords.pose.position.z = 0.0;

            ppl.push_back(poseInTargetCoords.pose.position);
//            ids.push_back(pt.id);
            ids.push_back(0);
//            uuids.push_back(pt.uuid);
            uuids.push_back("");
//            scores.push_back(pt.score);
            scores.push_back(0);
            std::vector<double> polar = cartesianToPolar(poseInRobotCoords.pose.position);
            distances.push_back(polar[0]);
            angles.push_back(polar[1]);

            angle = polar[0] < min_dist ? polar[1] : angle;
            closest_person = polar[0] < min_dist ? poseInRobotCoords : closest_person;
            min_dist = polar[0] < min_dist ? polar[0] : min_dist;
        }
    }
    if(ppl.size())
        st->addObservation(ppl, pta->header.stamp.toSec());
    publishDetections(pta->header, ppl, ids, uuids, scores, distances, angles, min_dist, angle);
    pub_pose.publish(closest_person);
    createVisualisation(ppl, test_marker);
//    if(pub_marker.getNumSubscribers())
//        createVisualisation(ppl);
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void PeopleTracker::connectCallback(ros::NodeHandle &n, ros::Subscriber &sub, std::string topic) {
    bool loc = pub_detect.getNumSubscribers();
    bool markers = pub_marker.getNumSubscribers();
    bool test_markers = pub_marker.getNumSubscribers();
    if(!loc && !markers && !test_marker) {
        ROS_DEBUG("Pedestrian Localisation: No subscribers. Unsubscribing.");
        sub.shutdown();
    } else {
        ROS_DEBUG("Pedestrian Localisation: New subscribers. Subscribing.");
        sub = n.subscribe(topic.c_str(), 10, &PeopleTracker::detectorCallback, this);
    }
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "people_tracker");
    PeopleTracker* pl = new PeopleTracker();
    return 0;
}
