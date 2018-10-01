#include "bayes_people_tracker/people_tracker.h"
#include <tf/transform_datatypes.h>
#include <XmlRpc.h>


PeopleTracker::PeopleTracker() : detect_seq(0), marker_seq(0)
{
    ros::NodeHandle n;

    listener = new tf::TransformListener();

    startup_time = ros::Time::now().toSec();
    startup_time_str = num_to_str<double>(startup_time);

    // Declare variables that can be modified by launch file or command line.
    std::string pta_topic;
    std::string pub_topic;
    std::string pub_topic_pose;
    std::string pub_topic_pose_array;
    std::string pub_topic_people;
    std::string pub_topic_trajectory;
    std::string pub_marker_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle("~");
    private_node_handle.param("target_frame", target_frame, std::string("/base_link"));
    private_node_handle.param("base_frame", base_frame, std::string("/base_link"));
    private_node_handle.param("people_array", pta_topic, std::string("/upper_body_detector/bounding_box_centres"));
    private_node_handle.param("tracker_frequency", tracker_frequency, double(30.0));
    parseParams(private_node_handle);

    // Create a status callback.
    ros::SubscriberStatusCallback con_cb = boost::bind(&PeopleTracker::connectCallback, this, boost::ref(n));

    private_node_handle.param("positions", pub_topic, std::string("/people_tracker/positions"));
    pub_detect = n.advertise<bayes_people_tracker::PeopleTracker>(pub_topic.c_str(), 100, con_cb, con_cb);
    private_node_handle.param("pose", pub_topic_pose, std::string("/people_tracker/pose"));
    pub_pose = n.advertise<geometry_msgs::PoseStamped>(pub_topic_pose.c_str(), 100, con_cb, con_cb);
    private_node_handle.param("pose_array", pub_topic_pose_array, std::string("/people_tracker/pose_array"));
    pub_pose_array = n.advertise<geometry_msgs::PoseArray>(pub_topic_pose_array.c_str(), 100, con_cb, con_cb);
    private_node_handle.param("people", pub_topic_people, std::string("/people_tracker/people"));
    pub_people = n.advertise<people_msgs::People>(pub_topic_people.c_str(), 100, con_cb, con_cb);
    private_node_handle.param("trajectory", pub_topic_trajectory, std::string("/people_tracker/trajectory"));
    pub_trajectory = n.advertise<geometry_msgs::PoseArray>(pub_topic_trajectory.c_str(), 100, con_cb, con_cb);
    private_node_handle.param("marker", pub_marker_topic, std::string("/people_tracker/marker_array"));
    pub_marker = n.advertise<visualization_msgs::MarkerArray>(pub_marker_topic.c_str(), 100, con_cb, con_cb);

    boost::thread tracking_thread(boost::bind(&PeopleTracker::trackingThread, this));

    ros::spin();
}

void PeopleTracker::parseParams(ros::NodeHandle n) {
    std::string filter;
    n.getParam("filter_type", filter);
    ROS_INFO_STREAM("Found filter type: " << filter);

    float stdLimit = 1.0;
    if (n.hasParam("std_limit")) {
        n.getParam("std_limit", stdLimit);
        ROS_INFO_STREAM("std_limit pruneTracks with " << stdLimit);       
    }

    bool prune_named = false;
    if (n.hasParam("prune_named")) {
        n.getParam("prune_named", prune_named);
        ROS_INFO_STREAM("prune_named with " << prune_named);       
    }


    if (filter == "EKF") {
        ekf = new SimpleTracking<EKFilter>(stdLimit, prune_named);
    } else if (filter == "UKF") {
        ukf = new SimpleTracking<UKFilter>(stdLimit, prune_named);
    } else if (filter == "PF") {
        pf = new SimpleTracking<PFilter>(stdLimit, prune_named);
    } else {
        ROS_FATAL_STREAM("Filter type " << filter << " is not specified. Unable to create the tracker. Please use either EKF, UKF or PF.");
        return;
    }

    XmlRpc::XmlRpcValue cv_noise;
    n.getParam("cv_noise_params", cv_noise);
    ROS_ASSERT(cv_noise.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_INFO_STREAM("Constant Velocity Model noise: " << cv_noise);
    if (ekf != NULL) {
        ekf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
    } else if (ukf != NULL) {
        ukf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
    } else if (pf != NULL) {
        pf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
    } else {
        ROS_FATAL_STREAM("no filter configured.");
    }
    ROS_INFO_STREAM("Created " << filter << " based tracker using constant velocity prediction model.");

    XmlRpc::XmlRpcValue detectors;
    n.getParam("detectors", detectors);
    ROS_ASSERT(detectors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = detectors.begin(); it != detectors.end(); ++it) {
        ROS_INFO_STREAM("Found detector: " << (std::string)(it->first) << " ==> " << detectors[it->first]);
        observ_model_t om_flag;
        double pos_noise_x = .2;
        double pos_noise_y = .2;
        int seq_size = 5;
        double seq_time = 0.2;
        association_t association = NN;
        om_flag = CARTESIAN;

        try {
            if (detectors[it->first].hasMember("seq_size"))
                seq_size = (int) detectors[it->first]["seq_size"];
            if (detectors[it->first].hasMember("seq_time"))
                seq_time = (double) detectors[it->first]["seq_time"];
            if (detectors[it->first].hasMember("matching_algorithm"))
                association = detectors[it->first]["matching_algorithm"] == "NN" ? NN 
                    : detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA 
                    : detectors[it->first]["matching_algorithm"] == "NN_LABELED" ? NN_LABELED 
                    : throw(asso_exception());
            if (detectors[it->first].hasMember("cartesian_noise_params")) { // legacy support
                pos_noise_x = detectors[it->first]["cartesian_noise_params"]["x"];
                pos_noise_y = detectors[it->first]["cartesian_noise_params"]["y"];
            }
            if (detectors[it->first].hasMember("noise_params")) {
                pos_noise_x = detectors[it->first]["noise_params"]["x"];
                pos_noise_y = detectors[it->first]["noise_params"]["y"];
            }
        } catch (XmlRpc::XmlRpcException& e) {
            ROS_FATAL_STREAM("XmlRpc::XmlRpcException: '"
                   << e.getMessage()
                   << "'\n"
                   << "Failed to parse definition for '"
                   << (std::string)(it->first)
                   << "'. Check your parameters."
                   );
            throw(e);

        }

        try {
            if (ekf != NULL) {
                ekf->addDetectorModel(
                    it->first,
                    association,
                    om_flag,
                    pos_noise_x, pos_noise_y,
                    seq_size, seq_time
                    );
            } else if (ukf != NULL) {
                ukf->addDetectorModel(
                    it->first,
                    association,
                    om_flag,
                    pos_noise_x, pos_noise_y,
                    seq_size, seq_time
                    );                
            } else if (pf != NULL) {
                pf->addDetectorModel(
                    it->first,
                    association,
                    om_flag,
                    pos_noise_x, pos_noise_y,
                    seq_size, seq_time
                    );                
            }
        } catch (asso_exception& e) {
            ROS_FATAL_STREAM(""
                   << e.what()
                   << " "
                   << detectors[it->first]["matching_algorithm"]
                   << " is not specified. Unable to add "
                   << (std::string)(it->first)
                   << " to the tracker. Please use either NN or NNJPDA as association algorithms."
                   );
            return;
        } catch (observ_exception& e) {
            ROS_FATAL_STREAM(""
                   << e.what()
                   << " "
                   << detectors[it->first]["observation_model"]
                   << " is not specified. Unable to add "
                   << (std::string)(it->first)
                   << " to the tracker. Please use either CARTESIAN or POLAR as observation models."
                   );
                return;
        }
        ros::Subscriber sub;
        if (detectors[it->first].hasMember("topic")) {
            subscribers[std::pair<std::string, std::string>(it->first, detectors[it->first]["topic"])] = sub;
        } 
        else if (detectors[it->first].hasMember("people_topic")) {
            subscribers_people[std::pair<std::string, std::string>(it->first, detectors[it->first]["people_topic"])] = sub;
        }
    }
}


void PeopleTracker::trackingThread() {
    ros::Rate fps(tracker_frequency);
    double time_sec = 0.0;

    while(ros::ok()) {
        std::map<long, std::string> tags;
        try {
            std::map<long, std::vector<geometry_msgs::Pose> > ppl;
            if (ekf != NULL) { 
                ppl = ekf->track(&time_sec, tags);
            } else if (ukf != NULL) {
                ppl = ukf->track(&time_sec, tags);
            } else if (pf != NULL) {
                ppl = pf->track(&time_sec, tags);
            }

            if(ppl.size()) {
                geometry_msgs::Pose closest_person_point;
                std::vector<geometry_msgs::Pose> poses;
                std::vector<geometry_msgs::Pose> vels;
                std::vector<geometry_msgs::Pose> vars;
                std::vector<std::string> uuids;
                std::vector<long> pids;
                std::vector<double> distances;
                std::vector<double> angles;
                double min_dist = DBL_MAX;
                double angle;

                for(std::map<long, std::vector<geometry_msgs::Pose> >::const_iterator it = ppl.begin();
                    it != ppl.end(); ++it) {
                    poses.push_back(it->second[0]);
                    vels.push_back(it->second[1]);
                    vars.push_back(it->second[2]);
                    if (tags[it->first] == "")
                        uuids.push_back(generateUUID(startup_time_str, it->first));
                    else
                        uuids.push_back(tags[it->first]);
                    pids.push_back(it->first);

                    geometry_msgs::PoseStamped poseInRobotCoords;
                    geometry_msgs::PoseStamped poseInTargetCoords;
                    poseInTargetCoords.header.frame_id = target_frame;
                    poseInTargetCoords.header.stamp.fromSec(time_sec);
                    poseInTargetCoords.pose = it->second[0];

                    //Find closest person and get distance and angle
                    if(strcmp(target_frame.c_str(), base_frame.c_str())) {
                        try{
                            ROS_DEBUG("Transforming received position into %s coordinate system.", base_frame.c_str());
                            listener->waitForTransform(poseInTargetCoords.header.frame_id, base_frame, poseInTargetCoords.header.stamp, ros::Duration(3.0));
                            listener->transformPose(base_frame, ros::Time(0), poseInTargetCoords, poseInTargetCoords.header.frame_id, poseInRobotCoords);
                        } catch(tf::TransformException ex) {
                            ROS_WARN("Failed transform: %s", ex.what());
                            continue;
                        }
                    } else {
                        poseInRobotCoords = poseInTargetCoords;
                    }

            	    if(pub_detect.getNumSubscribers() || pub_pose.getNumSubscribers() || pub_pose_array.getNumSubscribers() || pub_people.getNumSubscribers()) {
                        std::vector<double> polar = cartesianToPolar(poseInRobotCoords.pose.position);
                        distances.push_back(polar[0]);
                        angles.push_back(polar[1]);
                        angle = polar[0] < min_dist ? polar[1] : angle;
                        closest_person_point = polar[0] < min_dist ? it->second[0] : closest_person_point;
                        min_dist = polar[0] < min_dist ? polar[0] : min_dist;
                    }
                }

                if(pub_detect.getNumSubscribers() || pub_pose.getNumSubscribers() || pub_pose_array.getNumSubscribers() || pub_people.getNumSubscribers())
        	        publishDetections(time_sec, closest_person_point, poses, vels, uuids, distances, angles, min_dist, angle);

                if(pub_marker.getNumSubscribers())
                    createVisualisation(poses, vars, pids, pub_marker, uuids);

                //if(pub_trajectory.getNumSubscribers())
                publishTrajectory(poses, vels, vars, pids, pub_trajectory);
            }
            fps.sleep();
        }
        catch(std::exception& e) {
            ROS_INFO_STREAM("Exception: " << e.what());
            fps.sleep();
        }
        catch(Bayesian_filter::Numeric_exception& e) {
            ROS_INFO_STREAM("Exception: " << e.what());
            fps.sleep();
        }
    }
}

void PeopleTracker::publishDetections(
        double time_sec,
        geometry_msgs::Pose closest,
        std::vector<geometry_msgs::Pose> ppl,
        std::vector<geometry_msgs::Pose> vels,
        std::vector<std::string> uuids,
        std::vector<double> distances,
        std::vector<double> angles,
        double min_dist,
        double angle) {
    bayes_people_tracker::PeopleTracker result;
    result.header.stamp.fromSec(time_sec);
    result.header.frame_id = target_frame;
    result.header.seq = ++detect_seq;
    result.poses = ppl;
    result.uuids = uuids;
    result.distances = distances;
    result.angles = angles;
    result.min_distance = min_dist;
    result.min_distance_angle = angle;

    people_msgs::People people;
    people.header = result.header;
    for(int i = 0; i < ppl.size(); i++) {
        // Just running one loop for people_msgs and adding velocities to people_tracker message
        // Adding velocities as a vector to PeopleTracker message
        geometry_msgs::Vector3 v;
        v.x = vels[i].position.x;
        v.y = vels[i].position.y;
        result.velocities.push_back(v);

        // Creating and adding Person message
        people_msgs::Person person;
        person.position = ppl[i].position;
        person.velocity = vels[i].position;
        person.name = uuids[i];
        person.tags.push_back(uuids[i]);
        person.tagnames.push_back("uuid");
        person.reliability = 1.0;
        people.people.push_back(person);
    }

    // Publishing both messages
    publishDetections(result);
    publishDetections(people);

    geometry_msgs::PoseStamped pose;
    pose.header = result.header;
    pose.pose = closest;
    publishDetections(pose);

    geometry_msgs::PoseArray poses;
    poses.header = result.header;
    poses.poses = ppl;
    publishDetections(poses);
}

void PeopleTracker::publishDetections(bayes_people_tracker::PeopleTracker msg) {
    pub_detect.publish(msg);
}

void PeopleTracker::publishDetections(geometry_msgs::PoseStamped msg) {
    pub_pose.publish(msg);
}

void PeopleTracker::publishDetections(geometry_msgs::PoseArray msg) {
    pub_pose_array.publish(msg);
}

void PeopleTracker::publishDetections(people_msgs::People msg) {
    pub_people.publish(msg);
}

void PeopleTracker::publishTrajectory(std::vector<geometry_msgs::Pose> poses,
                      std::vector<geometry_msgs::Pose> vels,
                      std::vector<geometry_msgs::Pose> vars,
                      std::vector<long> pids,
                      ros::Publisher& pub) {
 /*** find trajectories ***/
 for(int i = 0; i < previous_poses.size(); i++) {
   if(boost::get<0>(previous_poses[i]) != INVALID_ID) {
     bool last_pose = true;
     for(int j = 0; j < pids.size(); j++) {
     if(pids[j] == boost::get<0>(previous_poses[i])) {
       last_pose = false;
       break;
     }
     }
     if(last_pose) {
     geometry_msgs::PoseArray trajectory;
     geometry_msgs::PoseArray velocity;
     geometry_msgs::PoseArray variance;
     trajectory.header.seq = boost::get<0>(previous_poses[i]); // tracking ID
     trajectory.header.stamp = ros::Time::now();
     trajectory.header.frame_id = target_frame; // will be reused by P-N experts
     for(int j = 0; j < previous_poses.size(); j++) {
       if(boost::get<0>(previous_poses[j]) == trajectory.header.seq) {
         trajectory.poses.push_back(boost::get<3>(previous_poses[j]));
         velocity.poses.push_back(boost::get<2>(previous_poses[j]));
         variance.poses.push_back(boost::get<1>(previous_poses[j]));
         boost::get<0>(previous_poses[j]) = INVALID_ID;
       }
     }
     pub.publish(trajectory);
     //std::cerr << "[people_tracker] trajectory ID = " << trajectory.header.seq << ", timestamp = " << trajectory.header.stamp << ", poses size = " << trajectory.poses.size() << std::endl;
     }
   }
 }

 /*** clean up ***/
 for(int i = 0; i < previous_poses.size(); i++) {
   if(boost::get<0>(previous_poses[i]) == INVALID_ID)
     previous_poses.erase(previous_poses.begin()+i);
 }
}

void PeopleTracker::createVisualisation(std::vector<geometry_msgs::Pose> poses, std::vector<geometry_msgs::Pose> vars, std::vector<long> pids, ros::Publisher& pub, std::vector<std::string> uuids) {
    ROS_DEBUG("Creating markers");
    visualization_msgs::MarkerArray marker_array;
    for(int i = 0; i < poses.size(); i++) {
        // Create Human Model
        std::vector<visualization_msgs::Marker> human = pm.createHuman(i*10, poses[i], target_frame);
        marker_array.markers.insert(marker_array.markers.begin(), human.begin(), human.end());
        // Create ID marker and trajectory
        double human_height = 1.9; //meter
        visualization_msgs::Marker tracking_id;
        tracking_id.header.stamp = ros::Time::now();
        tracking_id.header.frame_id = target_frame;
        tracking_id.ns = "people_id";
        tracking_id.id = pids[i];
        tracking_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        tracking_id.pose.position.x = poses[i].position.x;
        tracking_id.pose.position.y = poses[i].position.y;
        tracking_id.pose.position.z = human_height;
        tracking_id.scale.z = 0.7;
        tracking_id.color.a = 1.0;
        tracking_id.color.r = 1.0;
        tracking_id.color.g = 0.2;
        tracking_id.color.b = 0.0;
        tracking_id.text = uuids[i] + " (" + boost::to_string(pids[i]) + ")";
        tracking_id.lifetime = ros::Duration(0.1);
        marker_array.markers.push_back(tracking_id);

        visualization_msgs::Marker vars_ellipse;
        vars_ellipse.header.stamp = ros::Time::now();
        vars_ellipse.header.frame_id = target_frame;
        vars_ellipse.ns = "vars_ellipse";
        vars_ellipse.id = pids[i];
        vars_ellipse.type = visualization_msgs::Marker::CYLINDER;
        vars_ellipse.pose.position.x = poses[i].position.x;
        vars_ellipse.pose.position.y = poses[i].position.y;
        vars_ellipse.pose.position.z = 0.0;
        vars_ellipse.scale.z = 0.2;
        vars_ellipse.scale.x = sqrt(vars[i].position.x);
        vars_ellipse.scale.y = sqrt(vars[i].position.y);
        vars_ellipse.color.a = 0.8;
        vars_ellipse.color.r = 1.0 - (1.0/(sqrt(vars[i].position.x+vars[i].position.y)+1.0));
        vars_ellipse.color.g = (1.0/(sqrt(vars[i].position.x+vars[i].position.y)+1.0));
        vars_ellipse.color.b = 0.0;
        vars_ellipse.text = boost::to_string(vars[i].position.x) + ", " + boost::to_string(vars[i].position.y);
        //ROS_INFO_STREAM(vars_ellipse.text);
        vars_ellipse.lifetime = ros::Duration(0.1);
        marker_array.markers.push_back(vars_ellipse);


        /* for FLOBOT - tracking trajectory */
        visualization_msgs::Marker tracking_tr;
        tracking_tr.header.stamp = ros::Time::now();
        tracking_tr.header.frame_id = target_frame;
        tracking_tr.ns = "people_trajectory";
        tracking_tr.id = pids[i];
        tracking_tr.type = visualization_msgs::Marker::LINE_STRIP;
        geometry_msgs::Point p;
        for(int j = 0; j < previous_poses.size(); j++) {
            if(boost::get<0>(previous_poses[j]) == pids[i]) {
            	p.x = boost::get<3>(previous_poses[j]).position.x;
            	p.y = boost::get<3>(previous_poses[j]).position.y;
            	tracking_tr.points.push_back(p);
            }
        }
        tracking_tr.scale.x = 0.1;
        tracking_tr.color.a = 1.0;
        tracking_tr.color.r = std::max(0.3,(double)(pids[i]%3)/3.0);
        tracking_tr.color.g = std::max(0.3,(double)(pids[i]%6)/6.0);
        tracking_tr.color.b = std::max(0.3,(double)(pids[i]%9)/9.0);
        tracking_tr.lifetime = ros::Duration(1.0);
        marker_array.markers.push_back(tracking_tr);
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

void PeopleTracker::detectorCallback(const geometry_msgs::PoseArray::ConstPtr &pta, std::string detector)
{
    // Publish an empty message to trigger callbacks even when there are no detections.
    // This can be used by nodes which might also want to know when there is no human detected.
    if(pta->poses.size() == 0) {
        bayes_people_tracker::PeopleTracker empty;
        empty.header.stamp = ros::Time::now();
        empty.header.frame_id = target_frame;
        empty.header.seq = ++detect_seq;
        publishDetections(empty);
        return;
    }

    std::vector<geometry_msgs::Point> ppl;
    for(int i = 0; i < pta->poses.size(); i++) {
        geometry_msgs::Pose pt = pta->poses[i];

            //Create stamped pose for tf
            geometry_msgs::PoseStamped poseInCamCoords;
            geometry_msgs::PoseStamped poseInTargetCoords;
            poseInCamCoords.header = pta->header;
            poseInCamCoords.pose = pt;

            if (target_frame == poseInCamCoords.header.frame_id) {
                poseInTargetCoords = poseInCamCoords;
            } else {
            //Transform
                try {
                    // Transform into given traget frame. Default /map
                    ROS_DEBUG("Transforming received position into %s coordinate system.", target_frame.c_str());
                    listener->waitForTransform(poseInCamCoords.header.frame_id, target_frame, poseInCamCoords.header.stamp, ros::Duration(3.0));
                    listener->transformPose(target_frame, ros::Time(0), poseInCamCoords, poseInCamCoords.header.frame_id, poseInTargetCoords);
                }
                catch(tf::TransformException ex) {
                    ROS_WARN("Failed transform: %s", ex.what());
                    return;
                }
            }

            poseInTargetCoords.pose.position.z = 0.0;
            ppl.push_back(poseInTargetCoords.pose.position);

    }
  if(ppl.size()) {
    if(ekf == NULL) {
      if(ukf == NULL) {
	pf->addObservation(detector, ppl, pta->header.stamp.toSec(), std::vector<std::string>());
      } else {
	ukf->addObservation(detector, ppl, pta->header.stamp.toSec(), std::vector<std::string>());
      }
    } else {
      ekf->addObservation(detector, ppl, pta->header.stamp.toSec(), std::vector<std::string>());
    }
  }
}

void PeopleTracker::detectorCallback_people(const people_msgs::People::ConstPtr &people, std::string detector)
{
    // Publish an empty message to trigger callbacks even when there are no detections.
    // This can be used by nodes which might also want to know when there is no human detected.
    if(people->people.size() == 0) {
        bayes_people_tracker::PeopleTracker empty;
        empty.header.stamp = ros::Time::now();
        empty.header.frame_id = target_frame;
        empty.header.seq = ++detect_seq;
        publishDetections(empty);
        return;
    }

    std::vector<geometry_msgs::Point> ppl;
    std::vector<std::string> tags;
    for(int i = 0; i < people->people.size(); i++) {
        people_msgs::Person pt = people->people[i];

        //Create stamped pose for tf
        geometry_msgs::PoseStamped poseInCamCoords;
        geometry_msgs::PoseStamped poseInTargetCoords;
        poseInCamCoords.header = people->header;

        poseInCamCoords.pose.position = pt.position;
        tf::Quaternion temp_quat;
        temp_quat.setRPY( 0, 0, 0 ); // detections don't have an orientation
        tf::quaternionTFToMsg(temp_quat, poseInCamCoords.pose.orientation);

        if (target_frame == poseInCamCoords.header.frame_id) {
            poseInTargetCoords = poseInCamCoords;
        } else {
        //Transform
            try {
                // Transform into given traget frame. Default /map
                ROS_DEBUG("Transforming received position into %s coordinate system.", target_frame.c_str());
                listener->waitForTransform(poseInCamCoords.header.frame_id, target_frame, poseInCamCoords.header.stamp, ros::Duration(3.0));
                listener->transformPose(target_frame, ros::Time(0), poseInCamCoords, poseInCamCoords.header.frame_id, poseInTargetCoords);
            }
            catch(tf::TransformException ex) {
                ROS_WARN("Failed transform: %s", ex.what());
                return;
            }
        }

        poseInTargetCoords.pose.position.z = 0.0;
        ppl.push_back(poseInTargetCoords.pose.position);
        tags.push_back(pt.name);

    }
  if(ppl.size()) {
    if(ekf == NULL) {
      if(ukf == NULL) {
    pf->addObservation(detector, ppl, people->header.stamp.toSec(), tags);
      } else {
    ukf->addObservation(detector, ppl, people->header.stamp.toSec(), tags);
      }
    } else {
      ekf->addObservation(detector, ppl, people->header.stamp.toSec(), tags);
    }
  }
}


// Connection callback that unsubscribes from the tracker if no one is subscribed.
void PeopleTracker::connectCallback(ros::NodeHandle &n) {
    bool loc = pub_detect.getNumSubscribers();
    bool markers = pub_marker.getNumSubscribers();
    bool people = pub_people.getNumSubscribers();
    bool pose = pub_pose.getNumSubscribers();
    bool pose_array = pub_pose_array.getNumSubscribers();
    bool trajectory = pub_trajectory.getNumSubscribers();
    std::map<std::pair<std::string, std::string>, ros::Subscriber>::const_iterator it;
    if(!loc && !markers && !trajectory && !pose && !pose_array) {
        ROS_DEBUG("Pedestrian Localisation: No subscribers. Unsubscribing.");
        for(it = subscribers.begin(); it != subscribers.end(); ++it)
            const_cast<ros::Subscriber&>(it->second).shutdown();
    } else {
        ROS_DEBUG("Pedestrian Localisation: New subscribers. Subscribing.");
        for(it = subscribers.begin(); it != subscribers.end(); ++it)
            subscribers[it->first] = n.subscribe<geometry_msgs::PoseArray>(it->first.second.c_str(), 1000, boost::bind(&PeopleTracker::detectorCallback, this, _1, it->first.first));
        for(it = subscribers_people.begin(); it != subscribers_people.end(); ++it)
            subscribers_people[it->first] = n.subscribe<people_msgs::People>(it->first.second.c_str(), 1000, boost::bind(&PeopleTracker::detectorCallback_people, this, _1, it->first.first));
    }
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "bayes_people_tracker");
    PeopleTracker* pl = new PeopleTracker();
    return 0;
}
