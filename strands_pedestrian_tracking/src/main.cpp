// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <QImage>
#include <QPainter>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>

#include <string.h>
#include <boost/thread.hpp>

#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>

#include "Matrix.h"
#include "Vector.h"
#include "Camera.h"
#include "Globals.h"
#include "Hypo.h"
#include "Detections.h"
#include "AncillaryMethods.h"
#include "Tracker.h"

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

ros::Publisher pub_message;
image_transport::Publisher pub_image;

cv::Mat img_depth_;
cv_bridge::CvImagePtr cv_depth_ptr;	// cv_bridge for depth image

//string path_config_file = "/home/mitzel/Desktop/sandbox/Demo/upper_and_cuda/bin/config_Asus.inp";

Vector< Hypo > HyposAll;
Detections *det_comb;
Tracker tracker;
int cnt = 0;

//CImgDisplay* main_disp;
CImg<unsigned char> cim(640,480,1,3);

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

void ReadConfigParams(ros::NodeHandle n)
{
    std::string ns = "/pedestrian_tracking/";

    //=====================================
    // Input paths
    //=====================================
    n.getParam(ns+"camPath_left", Globals::camPath_left);
    n.getParam(ns+"sImagePath_left", Globals::sImagePath_left);
    n.getParam(ns+"tempDepthL", Globals::tempDepthL);
    n.getParam(ns+"path_to_planes", Globals::path_to_planes);

    //=====================================
    // Distance Range Accepted Detections
    //=====================================
    n.param(ns+"distance_range_accepted_detections", Globals::distance_range_accepted_detections, double(7));

    //======================================
    // ROI
    //======================================
    n.getParam(ns+"inc_width_ratio", Globals::inc_width_ratio);
    n.getParam(ns+"inc_height_ratio", Globals::inc_height_ratio);
    n.param(ns+"region_size_threshold", Globals::region_size_threshold, int(10));

    //======================================
    // Freespace Parameters
    //======================================
    n.param(ns+"freespace_scaleZ", Globals::freespace_scaleZ, double(20));
    n.param(ns+"freespace_scaleX", Globals::freespace_scaleX, double(20));
    n.param(ns+"freespace_minX", Globals::freespace_minX, double(-20));
    n.param(ns+"freespace_minZ", Globals::freespace_minZ, double(0));
    n.param(ns+"freespace_maxX", Globals::freespace_maxX, double(20));
    n.param(ns+"freespace_maxZ", Globals::freespace_maxZ, double(30));
    n.param(ns+"freespace_threshold", Globals::freespace_threshold, double(120));
    n.param(ns+"freespace_max_depth_to_cons", Globals::freespace_max_depth_to_cons, int(20));

    //======================================
    // Evaluation Parameters
    //======================================
    n.param(ns+"evaluation_NMS_threshold", Globals::evaluation_NMS_threshold, double(0.4));
    n.param(ns+"evaluation_NMS_threshold_LM", Globals::evaluation_NMS_threshold_LM, double(0.4));
    n.param(ns+"evaluation_NMS_threshold_Border", Globals::evaluation_NMS_threshold_Border, double(0.4));
    n.param(ns+"evaluation_inc_height_ratio", Globals::evaluation_inc_height_ratio, double(0.2));
    n.param(ns+"evaluation_stride", Globals::evaluation_stride, int(3));
    n.param(ns+"evaluation_scale_stride", Globals::evaluation_scale_stride, double(1.03));
    n.param(ns+"evaluation_nr_scales", Globals::evaluation_nr_scales, int(1));
    n.param(ns+"evaluation_inc_cropped_height", Globals::evaluation_inc_cropped_height, int(20));
    n.param(ns+"evaluation_greedy_NMS_overlap_threshold", Globals::evaluation_greedy_NMS_overlap_threshold, double(0.1));
    n.param(ns+"evaluation_greedy_NMS_threshold", Globals::evaluation_greedy_NMS_threshold, double(0.25));
    //======================================
    // World scale
    //======================================
    n.getParam(ns+"WORLD_SCALE", Globals::WORLD_SCALE);

    //======================================
    // height and width of images
    //======================================
    n.getParam(ns+"dImHeight", Globals::dImHeight);
    n.getParam(ns+"dImWidth", Globals::dImWidth);

    //======================================
    // Camera
    //======================================
    n.getParam(ns+"baseline", Globals::baseline);

    //====================================
    // Number of Frames / offset
    //====================================
    n.getParam(ns+"numberFrames", Globals::numberFrames);
    n.getParam(ns+"nOffset", Globals::nOffset);

    //=====================================
    // Determines if save bounding boxes or not
    //=====================================
    n.param(ns+"export_bounding_box", Globals::export_bounding_box, bool(false));
    // Path of exported bounding boxes
    n.getParam(ns+"bounding_box_path", Globals::bounding_box_path);

    //=====================================
    // Determines if save result images or not
    //=====================================
    n.param(ns+"export_result_images", Globals::export_result_images, bool(false));
    n.getParam(ns+"result_images_path", Globals::result_images_path);

    //====================================
    // Size of Template
    //====================================
    n.getParam(ns+"template_size", Globals::template_size);


    /////////////////////////////////TRACKING PART/////////////////////////
    //======================================
    // Detections
    //======================================
    n.param(ns+"cutDetectionsUsingDepth", Globals::cutDetectionsUsingDepth, bool(false));

    n.getParam(ns+"frameRate", Globals::frameRate);

    //======================================
    // Camera
    //======================================
    n.getParam(ns+"farPlane", Globals::farPlane);

    //======================================
    // World scale
    //======================================
    n.getParam(ns+"binSize", Globals::binSize);

    //======================================
    // Pedestrians width and height
    //======================================
    n.getParam(ns+"pedSizeWVis", Globals::pedSizeWVis);
    n.getParam(ns+"pedSizeWCom", Globals::pedSizeWCom);
    n.getParam(ns+"pedSizeHCom", Globals::pedSizeHCom);

    //======================================
    // History
    //======================================
    n.getParam(ns+"history", Globals::history);

    //======================================
    // Pedestrians parameter
    //======================================
    n.getParam(ns+"dObjHeight", Globals::dObjHeight);
    n.getParam(ns+"dObjHVar", Globals::dObjHVar);

    //======================================
    // Adjustment for HOG detections
    //======================================
    n.getParam(ns+"cutHeightBBOXforColor", Globals::cutHeightBBOXforColor);
    n.getParam(ns+"cutWidthBBOXColor", Globals::cutWidthBBOXColor);
    n.getParam(ns+"posponeCenterBBOXColor", Globals::posponeCenterBBOXColor);

    //======================================
    // Thresholds for combining the detection from left and right camera
    //======================================
    n.getParam(ns+"probHeight", Globals::probHeight);

    //======================================
    // MDL parameters for trajectories
    //======================================
    n.getParam(ns+"k1", Globals::k1);
    n.getParam(ns+"k2", Globals::k2);
    n.getParam(ns+"k3", Globals::k3);
    n.getParam(ns+"k4", Globals::k4);

    //======================================
    // Threshold for distinction between static/moving object
    //======================================
    n.getParam(ns+"minvel", Globals::minvel);
    n.getParam(ns+"dMaxPedVel", Globals::dMaxPedVel);

    //======================================
    // Threshold for identity management
    //======================================
    n.getParam(ns+"dSameIdThresh", Globals::dSameIdThresh);

    //======================================
    // Trajectory
    //======================================
    n.getParam(ns+"threshLengthTraj", Globals::threshLengthTraj);

    //======================================
    // Thresholds for accepted and displayed hypotheses
    //======================================
    n.getParam(ns+"dTheta2", Globals::dTheta2);

    //======================================
    // Time ant for temporal decay
    //======================================
    n.getParam(ns+"dTau", Globals::dTau);

    //======================================
    // Time horizon for event cone search
    //======================================
    n.getParam(ns+"coneTimeHorizon", Globals::coneTimeHorizon);
    n.getParam(ns+"maxHoleLen", Globals::maxHoleLen);
    n.getParam(ns+"dHolePenalty", Globals::dHolePenalty);

    // Q - the system covariance
    n.getParam(ns+"sysUncX", Globals::sysUncX);
    n.getParam(ns+"sysUncY", Globals::sysUncY);
    n.getParam(ns+"sysUncRot", Globals::sysUncRot);
    n.getParam(ns+"sysUncVel", Globals::sysUncVel);
    n.getParam(ns+"sysUncAcc", Globals::sysUncAcc);

    n.getParam(ns+"kalmanObsMotionModelthresh", Globals::kalmanObsMotionModelthresh);
    n.getParam(ns+"kalmanObsColorModelthresh", Globals::kalmanObsColorModelthresh);

    /////////////////////////////////GP Estimator/////////////////////////
    n.getParam(ns+"nrInter_ransac", Globals::nrInter_ransac);
    n.getParam(ns+"numberOfPoints_reconAsObstacle", Globals::numberOfPoints_reconAsObstacle);

    //======================================
    // ROI Segmentation
    //======================================
    // Blurring parameters
    n.param(ns+"sigmaX", Globals::sigmaX, double(2.0));
    n.param(ns+"precisionX", Globals::precisionX, double(2.0));
    n.param(ns+"sigmaZ", Globals::sigmaZ, double(3.0));
    n.param(ns+"precisionZ", Globals::precisionZ, double(2.0));

    n.param(ns+"max_height", Globals::max_height, double(2.0));
    n.param(ns+"min_height", Globals::min_height, double(1.4));

    ///////////////////////////Recording /////////////////////
    n.param(ns+"from_camera", Globals::from_camera, bool(true));
    n.getParam(ns+"from_file_path", Globals::from_file_path);

    //////////////////////////Streaming///////////////////////
    n.getParam(ns+"stream_dest_IP", Globals::stream_dest_IP);

    ////////////////////////HOG Detector////////////////////////
    n.param(ns+"hog_max_scale", Globals::hog_max_scale, double(1.9));
    n.param(ns+"hog_score_thresh", Globals::hog_score_thresh, double(0.4));
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
    allHypoMsg.header = color->header;
    Vector<Vector<double> > trajPts;
    Vector<double> dir;
    for(int i = 0; i < hyposMDL.getSize(); i++)
    {
        PedestrianTracking oneHypoMsg;
        oneHypoMsg.header = color->header;
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
    allHypoMsg.header = color->header;
    Vector<Vector<double> > trajPts;
    Vector<double> dir;
    for(int i = 0; i < hyposMDL.getSize(); i++)
    {
        PedestrianTracking oneHypoMsg;
        oneHypoMsg.header = color->header;
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
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void connectCallback(message_filters::Subscriber<CameraInfo> &sub_cam,
                     message_filters::Subscriber<GroundPlane> &sub_gp,
                     message_filters::Subscriber<GroundHOGDetections> &sub_hog,
                     message_filters::Subscriber<UpperBodyDetector> &sub_ubd,
                     message_filters::Subscriber<VisualOdometry> &sub_vo,
                     image_transport::SubscriberFilter &sub_col,
                     image_transport::ImageTransport &it){
    if(!pub_message.getNumSubscribers() && !pub_image.getNumSubscribers()) {
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

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("queue_size", queue_size, int(10));
    private_node_handle_.param("config_file", config_file, string(""));

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

    ReadConfigParams(boost::ref(n));
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

    ros::spin();
    return 0;
}

