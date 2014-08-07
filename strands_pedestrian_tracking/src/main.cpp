// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>

#include <QImage>
#include <QPainter>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>

#include <string.h>
#include <sstream>
#include <boost/thread.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>

#include "Matrix.h"
#include "Vector.h"
#include "Camera.h"
#include "Globals.h"
#include "ConfigFile.h"
#include "Hypo.h"
#include "Detections.h"
#include "AncillaryMethods.h"
#include "Tracker.h"
#include "VisualisationMarkers.h"

#include "strands_perception_people_msgs/UpperBodyDetector.h"
#include "strands_perception_people_msgs/GroundPlane.h"
#include "strands_perception_people_msgs/GroundHOGDetections.h"
#include "strands_perception_people_msgs/VisualOdometry.h"
#include "strands_perception_people_msgs/PedestrianTracking.h"
#include "strands_perception_people_msgs/PedestrianTrackingArray.h"


using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace strands_perception_people_msgs;

ros::Publisher pub_message, pub_markers, pub_pose;
image_transport::Publisher pub_image;

tf::TransformListener* listener;
VisualisationMarkers* vm;

cv::Mat img_depth_;
cv_bridge::CvImagePtr cv_depth_ptr;	// cv_bridge for depth image

//string path_config_file = "/home/mitzel/Desktop/sandbox/Demo/upper_and_cuda/bin/config_Asus.inp";

Vector< Hypo > HyposAll;
Detections *det_comb;
Tracker tracker;
int cnt = 0;
double startup_time = 0.0;
std::string startup_time_str = "", target_frame;

//CImgDisplay* main_disp;
CImg<unsigned char> cim(640,480,1,3);

template<typename T>
std::string num_to_str(T num) {
    std::stringstream ss;
    ss << num;
    return ss.str();
}

Vector<double> fromCam2World(Vector<double> posInCamera, Camera cam)
{
    Matrix<double> rotMat = cam.get_R();

    Vector<double> posCam = cam.get_t();

    Matrix<double> trMat(4,4,0.0);
    trMat(3,3) = 1;
    trMat(0,0) = rotMat(0,0);
    trMat(0,1) = rotMat(0,1);
    trMat(0,2) = rotMat(0,2);
    trMat(1,0) = rotMat(1,0);
    trMat(1,1) = rotMat(1,1);
    trMat(1,2) = rotMat(1,2);
    trMat(2,0) = rotMat(2,0);
    trMat(2,1) = rotMat(2,1);
    trMat(2,2) = rotMat(2,2);

    posCam *= Globals::WORLD_SCALE;

    trMat(3,0) = posCam(0);
    trMat(3,1) = posCam(1);
    trMat(3,2) = posCam(2);

    Vector<double> transpoint = trMat*posInCamera;
    return transpoint;

}

double computeDepth(Vector<double> vbbox, Camera cam)
{
    Vector<double> pos3D;
    double distance;

    cam.bbToDetection(vbbox, pos3D, Globals::WORLD_SCALE, distance);
    Vector<double> posInCamCord(pos3D(0), pos3D(1), pos3D(2), 1);
    Vector<double> posInWorld = fromCam2World(posInCamCord, cam);
    return posInWorld(2);
}

void get_image(unsigned char* b_image, uint w, uint h, CImg<unsigned char>& cim)
{
    unsigned char* ptr = b_image;
    for (unsigned int row = 0; row < h; ++row)
    {
        for (unsigned int col = 0; col < w; ++col)
        {
            // access the viewerImage as column, row
            cim(col,row,0,0) = *(ptr++); // red component
            cim(col,row,0,1) = *(ptr++); // green
            cim(col,row,0,2) = *(ptr++); // blue
        }
    }
}

void ReadConfigFile(string path_config_file)
{

    ConfigFile config(path_config_file);

    //=====================================
    // Input paths
    //=====================================
    config.readInto(Globals::camPath_left, "camPath_left");
    config.readInto(Globals::sImagePath_left, "sImagePath_left");
    config.readInto(Globals::tempDepthL, "tempDepthL");
    config.readInto(Globals::path_to_planes, "path_to_planes");

    //=====================================
    // Distance Range Accepted Detections
    //=====================================
    Globals::distance_range_accepted_detections = config.read<double>("distance_range_accepted_detections", 7);

    //======================================
    // ROI
    //======================================
    Globals::inc_width_ratio = config.read<double>("inc_width_ratio");
    Globals::inc_height_ratio = config.read<double>("inc_height_ratio");
    Globals::region_size_threshold = config.read<double>("region_size_threshold", 10);

    //======================================
    // Freespace Parameters
    //======================================
    Globals::freespace_scaleZ = config.read<double>("freespace_scaleZ", 20);
    Globals::freespace_scaleX = config.read<double>("freespace_scaleX", 20);
    Globals::freespace_minX = config.read<double>("freespace_minX", -20);
    Globals::freespace_minZ = config.read<double>("freespace_minZ", 0);
    Globals::freespace_maxX = config.read<double>("freespace_maxX", 20);
    Globals::freespace_maxZ = config.read<double>("freespace_maxZ", 30);
    Globals::freespace_threshold = config.read<double>("freespace_threshold", 120);
    Globals::freespace_max_depth_to_cons = config.read<int>("freespace_max_depth_to_cons", 20);

    //======================================
    // Evaluation Parameters
    //======================================
    Globals::evaluation_NMS_threshold = config.read<double>("evaluation_NMS_threshold",0.4);
    Globals::evaluation_NMS_threshold_LM = config.read<double>("evaluation_NMS_threshold_LM",0.4);
    Globals::evaluation_NMS_threshold_Border = config.read<double>("evaluation_NMS_threshold_Border",0.4);
    Globals::evaluation_inc_height_ratio = config.read<double>("evaluation_inc_height_ratio",0.2);
    Globals::evaluation_stride = config.read<int>("evaluation_stride",3);
    Globals::evaluation_scale_stride = config.read<double>("evaluation_scale_stride",1.03);
    Globals::evaluation_nr_scales = config.read<int>("evaluation_nr_scales",1);
    Globals::evaluation_inc_cropped_height = config.read<int>("evaluation_inc_cropped_height",20);
    Globals::evaluation_greedy_NMS_overlap_threshold = config.read<double>("evaluation_greedy_NMS_overlap_threshold", 0.1);
    Globals::evaluation_greedy_NMS_threshold = config.read<double>("evaluation_greedy_NMS_threshold", 0.25);
    //======================================
    // World scale
    //======================================
    config.readInto(Globals::WORLD_SCALE, "WORLD_SCALE");

    //======================================
    // height and width of images
    //======================================
    Globals::dImHeight = config.read<int>("dImHeight");
    Globals::dImWidth = config.read<int>("dImWidth");

    //======================================
    // Camera
    //======================================
    Globals::baseline = config.read<double>("baseline");

    //====================================
    // Number of Frames / offset
    //====================================
    Globals::numberFrames = config.read<int>("numberFrames");
    Globals::nOffset = config.read<int>("nOffset");

    //======================================
    // Console output
    //======================================
    //Globals::verbose = config.read("verbose", false);

    //=====================================
    // Determines if save bounding boxes or not
    //=====================================
    Globals::export_bounding_box = config.read("export_bounding_box", false);
    // Path of exported bounding boxes
    config.readInto(Globals::bounding_box_path, "bounding_box_path");

    //=====================================
    // Determines if save result images or not
    //=====================================
    Globals::export_result_images = config.read("export_result_images", false);
    config.readInto(Globals::result_images_path, "result_images_path");

    //====================================
    // Size of Template
    //====================================
    Globals::template_size = config.read<int>("template_size");


    /////////////////////////////////TRACKING PART/////////////////////////
    //======================================
    // Detections
    //======================================
    Globals::cutDetectionsUsingDepth = config.read("cutDetectionsUsingDepth", false);

    Globals::frameRate = config.read<int>("frameRate");

    //======================================
    // Camera
    //======================================
    Globals::farPlane = config.read<double>("farPlane");

    //======================================
    // World scale
    //======================================
    config.readInto(Globals::binSize, "binSize");

    //======================================
    // Pedestrians width and height
    //======================================
    Globals::pedSizeWVis = config.read<double>("pedSizeWVis");
    Globals::pedSizeWCom = config.read<double>("pedSizeWCom");
    Globals::pedSizeHCom = config.read<double>("pedSizeHCom");

    //======================================
    // History
    //======================================
    Globals::history = config.read<int>("history");

    //======================================
    // Pedestrians parameter
    //======================================
    Globals::dObjHeight = config.read<double>("dObjHeight");
    Globals::dObjHVar = config.read<double>("dObjHVar");

    //======================================
    // Adjustment for HOG detections
    //======================================
    Globals::cutHeightBBOXforColor = config.read<double>("cutHeightBBOXforColor");
    Globals::cutWidthBBOXColor = config.read<double>("cutWidthBBOXColor");
    Globals::posponeCenterBBOXColor = config.read<double>("posponeCenterBBOXColor");

    //======================================
    // Thresholds for combining the detection from left and right camera
    //======================================
    Globals::probHeight = config.read<double>("probHeight");

    //======================================
    // Visualisation
    // Now handled by visualise parameter.
    //======================================
    //Globals::render_bbox3D = config.read("render_bbox3D", true);
    //Globals::render_bbox2D = config.read("render_bbox2D", false);
    //Globals::render_tracking_numbers = config.read("render_tracking_numbers", false);

    //======================================
    // MDL parameters for trajectories
    //======================================
    Globals::k1 = config.read<double>("k1");
    Globals::k2 = config.read<double>("k2");
    Globals::k3 = config.read<double>("k3");
    Globals::k4 = config.read<double>("k4");

    //======================================
    // Threshold for distinction between static/moving object
    //======================================
    Globals::minvel = config.read<double>("minvel");
    Globals::dMaxPedVel = config.read<double>("dMaxPedVel");

    //======================================
    // Threshold for identity management
    //======================================
    Globals::dSameIdThresh = config.read<double>("dSameIdThresh");

    //======================================
    // Trajectory
    //======================================
    Globals::threshLengthTraj = config.read<int>("threshLengthTraj");

    //======================================
    // Thresholds for accepted and displayed hypotheses
    //======================================
    Globals::dTheta2 = config.read<double>("dTheta2");

    //======================================
    // Time ant for temporal decay
    //======================================
    Globals::dTau = config.read<double>("dTau");

    //======================================
    // Time horizon for event cone search
    //======================================
    Globals::coneTimeHorizon = config.read<int>("coneTimeHorizon");
    Globals::maxHoleLen = config.read<int>("maxHoleLen");
    Globals::dHolePenalty = config.read<double>("dHolePenalty");

    // Q - the system covariance
    Globals::sysUncX = config.read<double>("sysUncX");
    Globals::sysUncY = config.read<double>("sysUncY");
    Globals::sysUncRot = config.read<double>("sysUncRot");
    Globals::sysUncVel = config.read<double>("sysUncVel");
    Globals::sysUncAcc = config.read<double>("sysUncAcc");

    Globals::kalmanObsMotionModelthresh = config.read<double>("kalmanObsMotionModelthresh");
    Globals::kalmanObsColorModelthresh = config.read<double>("kalmanObsColorModelthresh");

    /////////////////////////////////GP Estimator/////////////////////////
    Globals::nrInter_ransac = config.read<int>("nrInter_ransac");
    Globals::numberOfPoints_reconAsObstacle = config.read<int>("numberOfPoints_reconAsObstacle");

    //======================================
    // ROI Segmentation
    //======================================
    // Blurring parameters
    Globals::sigmaX = config.read<double>("sigmaX", 2.0);
    Globals::precisionX = config.read<double>("precisionX", 2.0);
    Globals::sigmaZ = config.read<double>("sigmaZ", 3.0);
    Globals::precisionZ = config.read<double>("precisionZ", 2.0);

    Globals::max_height = config.read<double>("max_height", 2.0);
    Globals::min_height = config.read<double>("min_height", 1.4);

    ///////////////////////////Recording /////////////////////
    Globals::from_camera = config.read("from_camera", true);
    config.readInto(Globals::from_file_path, "from_file_path");

    //////////////////////////Streaming///////////////////////
    config.readInto(Globals::stream_dest_IP, "stream_dest_IP");

    ////////////////////////HOG Detector////////////////////////
    Globals::hog_max_scale = config.read<float>("hog_max_scale",1.9);
    Globals::hog_score_thresh = config.read<float>("hog_score_thresh",0.4);
}

Camera createCamera(Vector<double>& GP,
                    const VisualOdometry::ConstPtr &vo,
                    const CameraInfoConstPtr &info) {
    Matrix<double> motion_matrix(4,4, (double*) (&vo->transformation_matrix[0]));
    Matrix<double> R(motion_matrix, 0,2,0,2);
    Vector<double> t(motion_matrix(3,0), motion_matrix(3,1), motion_matrix(3,2));
    Matrix<double> K(3,3, (double*)&info->K[0]);

    Camera camera(K, R, t, GP);
    Vector<double> GP_world = AncillaryMethods::PlaneToWorld(camera, GP);
    return Camera(K, R, t, GP_world);
}

std::string generateUUID(std::string time, int id) {
    boost::uuids::uuid dns_namespace_uuid;
    boost::uuids::name_generator gen(dns_namespace_uuid);
    time += num_to_str<int>(id);

    return num_to_str<boost::uuids::uuid>(gen(time.c_str()));
}

visualization_msgs::MarkerArray createVisualisation(geometry_msgs::PoseArray poses, std::string target_frame) {
    ROS_DEBUG("Creating markers");
    visualization_msgs::MarkerArray marker_array;
    for(int i = 0; i < poses.poses.size(); i++) {
        geometry_msgs::Pose pose = poses.poses[i];
        pose.position.z = 0.6;
        std::vector<visualization_msgs::Marker> human = vm->createHuman(i*10, target_frame, pose);
        marker_array.markers.insert(marker_array.markers.begin(), human.begin(), human.end());
    }
    return marker_array;
}

geometry_msgs::PoseArray transform(PedestrianTrackingArray pta) {
    geometry_msgs::PoseArray result;
    for(int i = 0; i < pta.pedestrians.size(); i++) {
        PedestrianTracking pt = pta.pedestrians[i];

        //Create stamped pose for tf
        geometry_msgs::PoseStamped poseInCamCoords;
        geometry_msgs::PoseStamped poseInTargetCoords;
        poseInCamCoords.header = pta.header;
        poseInCamCoords.pose.position.x = pt.traj_x_camera[0];
        poseInCamCoords.pose.position.y = pt.traj_y_camera[0];
        poseInCamCoords.pose.position.z = pt.traj_z_camera[0];

        //Counteracting rotation in tf because it is already done in the pedestrian tracking.
        poseInCamCoords.pose.orientation.x = -0.5;
        poseInCamCoords.pose.orientation.y =  0.5;
        poseInCamCoords.pose.orientation.z =  0.5;
        poseInCamCoords.pose.orientation.w =  0.5;

        //Transform
        try {
            // Transform into given traget frame. Default /map
            ROS_DEBUG("Transforming received position into %s coordinate system.", target_frame.c_str());
            listener->waitForTransform(poseInCamCoords.header.frame_id, target_frame, poseInCamCoords.header.stamp, ros::Duration(3.0));
            listener->transformPose(target_frame, ros::Time(0), poseInCamCoords, poseInCamCoords.header.frame_id, poseInTargetCoords);
        }
        catch(tf::TransformException ex) {
            ROS_WARN("Failed transform: %s", ex.what());
            return geometry_msgs::PoseArray();
        }
        result.poses.push_back(poseInTargetCoords.pose);
        if(result.header.seq == 0)
            result.header = poseInTargetCoords.header;
    }
    return result;
}

void callbackWithoutHOG(const ImageConstPtr &color,
                        const CameraInfoConstPtr &info,
                        const GroundPlane::ConstPtr &gp,
                        const UpperBodyDetector::ConstPtr &upper,
                        const VisualOdometry::ConstPtr &vo)
{
    ROS_DEBUG("Entered callback without groundHOG data");
    Globals::render_bbox3D = pub_image.getNumSubscribers() > 0 ? true : false;

    // Get camera from VO and GP
    Vector<double> GP(3, (double*) &gp->n[0]);
    GP.pushBack((double) gp->d);

    Camera camera = createCamera(GP, vo, info);

    // Get detections from upper body
    Vector<double> single_detection(9);
    Vector<Vector< double > > detected_bounding_boxes;

    for(int i = 0; i < upper->pos_x.size(); i++)
    {
        single_detection(0) = cnt;
        single_detection(1) = i;
        single_detection(2) = 1;
        single_detection(3) = 1 - upper->dist[i]; // make sure that the score is always positive
        single_detection(4) = upper->pos_x[i];
        single_detection(5) = upper->pos_y[i];
        single_detection(6) = upper->width[i];
        single_detection(7) = upper->height[i] * 3;
        single_detection(8) = upper->median_depth[i];
        detected_bounding_boxes.pushBack(single_detection);
    }

    get_image((unsigned char*)(&color->data[0]),info->width,info->height,cim);
    ///////////////////////////////////////////TRACKING///////////////////////////

    tracker.process_tracking_oneFrame(HyposAll, *det_comb, cnt, detected_bounding_boxes, cim, camera);
    Vector<Hypo> hyposMDL = tracker.getHyposMDL();


    PedestrianTrackingArray allHypoMsg;
    allHypoMsg.header = upper->header;
    Vector<Vector<double> > trajPts;
    Vector<double> dir;
    for(int i = 0; i < hyposMDL.getSize(); i++)
    {
        PedestrianTracking oneHypoMsg;
        oneHypoMsg.header = upper->header;
        hyposMDL(i).getTrajPts(trajPts);
        for(int j = 0; j < trajPts.getSize(); j++)
        {
            oneHypoMsg.traj_x.push_back(trajPts(j)(0));
            oneHypoMsg.traj_y.push_back(trajPts(j)(1));
            oneHypoMsg.traj_z.push_back(trajPts(j)(2));
            
            Vector<double> posInCamera = AncillaryMethods::fromWorldToCamera(trajPts(j), camera);
            
            oneHypoMsg.traj_x_camera.push_back(posInCamera(0));
            oneHypoMsg.traj_y_camera.push_back(posInCamera(1));
            oneHypoMsg.traj_z_camera.push_back(posInCamera(2));
        }

        oneHypoMsg.id = hyposMDL(i).getHypoID();
        oneHypoMsg.uuid = generateUUID(startup_time_str, oneHypoMsg.id);
        oneHypoMsg.score = hyposMDL(i).getScoreMDL();
        oneHypoMsg.speed = hyposMDL(i).getSpeed();
        hyposMDL(i).getDir(dir);

        oneHypoMsg.dir.push_back(dir(0));
        oneHypoMsg.dir.push_back(dir(1));
        oneHypoMsg.dir.push_back(dir(2));
        allHypoMsg.pedestrians.push_back(oneHypoMsg);
    }

    if(pub_image.getNumSubscribers()) {
        ROS_DEBUG("Publishing image");
        Image res_img;
        res_img.header = color->header;
        res_img.height = cim._height;
        res_img.width = cim._width;
        res_img.step   = color->step;
        for (std::size_t i = 0; i != cim._height*cim._width; ++i) {
            res_img.data.push_back(cim.data()[i+0*cim._height*cim._width]);
            res_img.data.push_back(cim.data()[i+1*cim._height*cim._width]);
            res_img.data.push_back(cim.data()[i+2*cim._height*cim._width]);
        }
        res_img.encoding = color->encoding;

        pub_image.publish(res_img);
    }

    pub_message.publish(allHypoMsg);
    cnt++;

    if(pub_markers.getNumSubscribers() || pub_pose.getNumSubscribers()){
        geometry_msgs::PoseArray p = transform(allHypoMsg);
        if(pub_pose.getNumSubscribers())
            pub_pose.publish(p);
        if(pub_markers.getNumSubscribers())
            pub_markers.publish(createVisualisation(p, target_frame));
    }
}

void callbackWithHOG(const ImageConstPtr &color,
                     const CameraInfoConstPtr &info,
                     const GroundPlane::ConstPtr &gp,
                     const GroundHOGDetections::ConstPtr& groundHOGDet,
                     const UpperBodyDetector::ConstPtr &upper,
                     const VisualOdometry::ConstPtr &vo)
{
    ROS_DEBUG("Entered callback with groundHOG data");
    Globals::render_bbox3D = pub_image.getNumSubscribers() > 0 ? true : false;

    // Get camera from VO and GP
    Vector<double> GP(3, (double*) &gp->n[0]);
    GP.pushBack((double) gp->d);

    Camera camera = createCamera(GP, vo, info);

    // Get detections from HOG and upper body
    Vector<double> single_detection(9);
    Vector<Vector< double > > detected_bounding_boxes;

    for(int i = 0; i < groundHOGDet->pos_x.size(); i++)
    {
        single_detection(0) = cnt;
        single_detection(1) = i;
        single_detection(2) = 1;
        single_detection(3) = groundHOGDet->score[i];
        single_detection(4) = groundHOGDet->pos_x[i];
        single_detection(5) = groundHOGDet->pos_y[i];
        single_detection(6) = groundHOGDet->width[i];
        single_detection(7) = groundHOGDet->height[i];
        Vector<double> bbox(single_detection(3), single_detection(4), single_detection(5), single_detection(6));
        single_detection(8) = computeDepth(bbox, camera);
        detected_bounding_boxes.pushBack(single_detection);
    }

    for(int i = 0; i < upper->pos_x.size(); i++)
    {
        single_detection(0) = cnt;
        single_detection(1) = groundHOGDet->pos_x.size()+i;
        single_detection(2) = 1;
        single_detection(3) = 1 - upper->dist[i]; // make sure that the score is always positive
        single_detection(4) = upper->pos_x[i];
        single_detection(5) = upper->pos_y[i];
        single_detection(6) = upper->width[i];
        single_detection(7) = upper->height[i] * 3;
        single_detection(8) = upper->median_depth[i];
        detected_bounding_boxes.pushBack(single_detection);
    }


    get_image((unsigned char*)(&color->data[0]),info->width,info->height,cim);
    ///////////////////////////////////////////TRACKING///////////////////////////

    tracker.process_tracking_oneFrame(HyposAll, *det_comb, cnt, detected_bounding_boxes, cim, camera);
    Vector<Hypo> hyposMDL = tracker.getHyposMDL();


    PedestrianTrackingArray allHypoMsg;
    allHypoMsg.header = upper->header;
    Vector<Vector<double> > trajPts;
    Vector<double> dir;
    for(int i = 0; i < hyposMDL.getSize(); i++)
    {
        PedestrianTracking oneHypoMsg;
        oneHypoMsg.header = upper->header;
        hyposMDL(i).getTrajPts(trajPts);
        for(int j = 0; j < trajPts.getSize(); j++)
        {
            oneHypoMsg.traj_x.push_back(trajPts(j)(0));
            oneHypoMsg.traj_y.push_back(trajPts(j)(1));
            oneHypoMsg.traj_z.push_back(trajPts(j)(2));
            
            Vector<double> posInCamera = AncillaryMethods::fromWorldToCamera(trajPts(j), camera);
            
            oneHypoMsg.traj_x_camera.push_back(posInCamera(0));
            oneHypoMsg.traj_y_camera.push_back(posInCamera(1));
            oneHypoMsg.traj_z_camera.push_back(posInCamera(2));
            
        }

        oneHypoMsg.id = hyposMDL(i).getHypoID();
        oneHypoMsg.uuid = generateUUID(startup_time_str, oneHypoMsg.id);
        oneHypoMsg.score = hyposMDL(i).getScoreMDL();
        oneHypoMsg.speed = hyposMDL(i).getSpeed();
        hyposMDL(i).getDir(dir);

        oneHypoMsg.dir.push_back(dir(0));
        oneHypoMsg.dir.push_back(dir(1));
        oneHypoMsg.dir.push_back(dir(2));
        allHypoMsg.pedestrians.push_back(oneHypoMsg);
    }

    if(pub_image.getNumSubscribers()) {
        ROS_DEBUG("Publishing image");
        Image res_img;
        res_img.header = color->header;
        res_img.height = cim._height;
        res_img.step   = color->step;
        res_img.width = cim._width;
        for (std::size_t i = 0; i != cim._height*cim._width; ++i) {
            res_img.data.push_back(cim.data()[i+0*cim._height*cim._width]);
            res_img.data.push_back(cim.data()[i+1*cim._height*cim._width]);
            res_img.data.push_back(cim.data()[i+2*cim._height*cim._width]);
        }
        res_img.encoding = color->encoding;

        pub_image.publish(res_img);
    }

    pub_message.publish(allHypoMsg);
    cnt++;

    if(strcmp(target_frame.c_str(), "") && (pub_markers.getNumSubscribers() || pub_pose.getNumSubscribers())){
        geometry_msgs::PoseArray p = transform(allHypoMsg);
        if(pub_pose.getNumSubscribers())
            pub_pose.publish(p);
        if(pub_markers.getNumSubscribers())
            pub_markers.publish(createVisualisation(p, target_frame));
    }
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void connectCallback(message_filters::Subscriber<CameraInfo> &sub_cam,
                     message_filters::Subscriber<GroundPlane> &sub_gp,
                     message_filters::Subscriber<GroundHOGDetections> &sub_hog,
                     message_filters::Subscriber<UpperBodyDetector> &sub_ubd,
                     message_filters::Subscriber<VisualOdometry> &sub_vo,
                     image_transport::SubscriberFilter &sub_col,
                     image_transport::ImageTransport &it){
    if(!pub_message.getNumSubscribers() && !pub_image.getNumSubscribers() && !pub_markers.getNumSubscribers() && !pub_pose.getNumSubscribers()) {
        ROS_DEBUG("Tracker: No subscribers. Unsubscribing.");
        sub_cam.unsubscribe();
        sub_gp.unsubscribe();
        sub_hog.unsubscribe();
        sub_ubd.unsubscribe();
        sub_vo.unsubscribe();
        sub_col.unsubscribe();
    } else {
        ROS_DEBUG("Tracker: New subscribers. Subscribing.");
        sub_cam.subscribe();
        sub_gp.subscribe();
        sub_hog.subscribe();
        sub_ubd.subscribe();
        sub_vo.subscribe();
        sub_col.subscribe(it,sub_col.getTopic().c_str(),1);
    }
}

int main(int argc, char **argv)
{
    Globals::render_bbox2D = false;
    Globals::render_tracking_numbers = false;

    // Set up ROS.
    ros::init(argc, argv, "pedestrian_tracking");
    ros::NodeHandle n;

    listener = new tf::TransformListener();
    vm = new VisualisationMarkers();

    startup_time = ros::Time::now().toSec();
    startup_time_str = num_to_str<double>(startup_time);

    // Declare variables that can be modified by launch file or command line.
    int queue_size;
    string config_file;
    string cam_ns;
    string topic_gp;
    string topic_groundHOG;
    string topic_upperbody;
    string topic_vo;

    string pub_topic;
    string pub_image_topic;
    string pub_marker_topic;
    string pub_pose_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("queue_size", queue_size, int(10));
    private_node_handle_.param("config_file", config_file, string(""));
    private_node_handle_.param("target_frame", target_frame, string("/map"));

    private_node_handle_.param("camera_namespace", cam_ns, string("/head_xtion"));
    private_node_handle_.param("ground_plane", topic_gp, string("/ground_plane"));
    private_node_handle_.param("ground_hog", topic_groundHOG, string("/groundHOG/detections"));
    private_node_handle_.param("upper_body_detections", topic_upperbody, string("/upper_body_detector/detections"));
    private_node_handle_.param("visual_odometry", topic_vo, string("/visual_odometry/motion_matrix"));

    string topic_color_image = cam_ns + "/rgb/image_rect_color";
    string topic_camera_info = cam_ns + "/rgb/camera_info";

    if(strcmp(config_file.c_str(),"") == 0) {
        ROS_ERROR("No config file specified.");
        ROS_ERROR("Run with: rosrun strands_pedestrian_tracking pedestrian_tracking _config_file:=/path/to/config");
        exit(0);
    }

    ReadConfigFile(config_file);
    det_comb = new Detections(23, 0);



    ROS_DEBUG("pedestrian_tracker: Queue size for synchronisation is set to: %i", queue_size);

    // Image transport handle
    image_transport::ImageTransport it(private_node_handle_);

    // Create a subscriber.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    // The immediate unsubscribe is necessary to start without subscribing to any topic because message_filters does nor allow to do it another way.
    image_transport::SubscriberFilter subscriber_color;
    subscriber_color.subscribe(it, topic_color_image.c_str(), 1); subscriber_color.unsubscribe(); //This subscribe and unsubscribe is just to set the topic name.
    message_filters::Subscriber<CameraInfo> subscriber_camera_info(n, topic_camera_info.c_str(), 1); subscriber_camera_info.unsubscribe();
    message_filters::Subscriber<GroundPlane> subscriber_gp(n, topic_gp.c_str(), 1); subscriber_gp.unsubscribe();
    message_filters::Subscriber<GroundHOGDetections> subscriber_groundHOG(n, topic_groundHOG.c_str(), 1); subscriber_groundHOG.unsubscribe();
    message_filters::Subscriber<UpperBodyDetector> subscriber_upperbody(n, topic_upperbody.c_str(), 1); subscriber_upperbody.unsubscribe();
    message_filters::Subscriber<VisualOdometry> subscriber_vo(n, topic_vo.c_str(), 1); subscriber_vo.unsubscribe();

    ros::SubscriberStatusCallback con_cb = boost::bind(&connectCallback,
                                                       boost::ref(subscriber_camera_info),
                                                       boost::ref(subscriber_gp),
                                                       boost::ref(subscriber_groundHOG),
                                                       boost::ref(subscriber_upperbody),
                                                       boost::ref(subscriber_vo),
                                                       boost::ref(subscriber_color),
                                                       boost::ref(it));
    image_transport::SubscriberStatusCallback image_cb = boost::bind(&connectCallback,
                                                                     boost::ref(subscriber_camera_info),
                                                                     boost::ref(subscriber_gp),
                                                                     boost::ref(subscriber_groundHOG),
                                                                     boost::ref(subscriber_upperbody),
                                                                     boost::ref(subscriber_vo),
                                                                     boost::ref(subscriber_color),
                                                                     boost::ref(it));

    ///////////////////////////////////////////////////////////////////////////////////
    //Registering callback
    ///////////////////////////////////////////////////////////////////////////////////
    // With groundHOG
    sync_policies::ApproximateTime<Image, CameraInfo, GroundPlane,
            GroundHOGDetections, UpperBodyDetector, VisualOdometry> MySyncPolicyHOG(queue_size); //The real queue size for synchronisation is set here.
    MySyncPolicyHOG.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.

    const sync_policies::ApproximateTime<Image, CameraInfo, GroundPlane,
            GroundHOGDetections, UpperBodyDetector, VisualOdometry> MyConstSyncPolicyHOG = MySyncPolicyHOG;

    Synchronizer< sync_policies::ApproximateTime<Image, CameraInfo, GroundPlane,
            GroundHOGDetections, UpperBodyDetector, VisualOdometry> >
            syncHOG(MyConstSyncPolicyHOG, subscriber_color, subscriber_camera_info, subscriber_gp,
                    subscriber_groundHOG, subscriber_upperbody, subscriber_vo);
    if(strcmp(topic_groundHOG.c_str(),"") != 0)
        syncHOG.registerCallback(boost::bind(&callbackWithHOG, _1, _2, _3, _4, _5, _6));
    ///////////////////////////////////////////////////////////////////////////////////
    // Without groundHOG
    sync_policies::ApproximateTime<Image, CameraInfo, GroundPlane,
            UpperBodyDetector, VisualOdometry> MySyncPolicy(queue_size); //The real queue size for synchronisation is set here.
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.

    const sync_policies::ApproximateTime<Image, CameraInfo, GroundPlane,
            UpperBodyDetector, VisualOdometry> MyConstSyncPolicy = MySyncPolicy;

    Synchronizer< sync_policies::ApproximateTime<Image, CameraInfo, GroundPlane,
            UpperBodyDetector, VisualOdometry> >
            sync(MyConstSyncPolicy, subscriber_color, subscriber_camera_info, subscriber_gp,
                 subscriber_upperbody, subscriber_vo);
    if(strcmp(topic_groundHOG.c_str(),"") == 0)
        sync.registerCallback(boost::bind(&callbackWithoutHOG, _1, _2, _3, _4, _5));
    ///////////////////////////////////////////////////////////////////////////////////

    // Create a topic publisher
    private_node_handle_.param("pedestrian_array", pub_topic, string("/pedestrian_tracking/pedestrian_array"));
    pub_message = n.advertise<PedestrianTrackingArray>(pub_topic.c_str(), 10, con_cb, con_cb);

    private_node_handle_.param("pedestrian_image", pub_image_topic, string("/pedestrian_tracking/image"));
    pub_image = it.advertise(pub_image_topic.c_str(), 1, image_cb, image_cb);

    private_node_handle_.param("pedestrian_markers", pub_marker_topic, string("/pedestrian_tracking/marker_array"));
    pub_markers = n.advertise<visualization_msgs::MarkerArray>(pub_marker_topic.c_str(), 10, con_cb, con_cb);

    private_node_handle_.param("pedestrian_poses", pub_pose_topic, string("/pedestrian_tracking/pose_array"));
    pub_pose = n.advertise<geometry_msgs::PoseArray>(pub_pose_topic.c_str(), 10, con_cb, con_cb);

    ros::spin();
    return 0;
}

