// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include "string.h"
#include "boost/thread.hpp"

#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>

#include "Matrix.h"
#include "Vector.h"
#include "Camera.h"
#include "pointcloud.h"
#include "detector.h"
#include "Globals.h"
#include "groundplaneestimator.h"
#include "ConfigFile.h"

#include "strands_perception_people_msgs/UpperBodyDetector.h"
#include "strands_perception_people_msgs/GroundPlane.h"

#include <QImage>
#include <QPainter>




using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

//sensor_msgs::CameraInfo* camera_info = NULL;
//boost::mutex camera_info_mutex;
/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
ros::Publisher pub_message;
ros::Publisher pub_result_image;
ros::Publisher pub_ground_plane;


cv::Mat img_depth_;
cv_bridge::CvImagePtr cv_depth_ptr;	// cv_bridge for depth image

GroundPlaneEstimator GPEstimator;
Matrix<double> upper_body_template;
Detector* detector;


//string path_config_file = "/home/mitzel/Desktop/sandbox/Demo/upper_and_cuda/bin/config_Asus.inp";


void render_bbox_2D(strands_perception_people_msgs::UpperBodyDetector& detections, QImage& image,
                    int r, int g, int b, int lineWidth)
{

    QPainter painter(&image);

    QColor qColor;
    qColor.setRgb(r, g, b);

    QPen pen;
    pen.setColor(qColor);
    pen.setWidth(lineWidth);

    painter.setPen(pen);

    for(int i = 0; i < detections.pos_x.size(); i++){
        int x =(int) detections.pos_x[i];
        int y =(int) detections.pos_y[i];
        int w =(int) detections.width[i];
        int h =(int) detections.height[i];

        painter.drawLine(x,y, x+w,y);
        painter.drawLine(x,y, x,y+h);
        painter.drawLine(x+w,y, x+w,y+h);
        painter.drawLine(x,y+h, x+w,y+h);
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
    Globals::verbose = config.read("verbose", false);

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
    //======================================
    Globals::render_bbox3D = config.read("render_bbox3D", true);
    Globals::render_bbox2D = config.read("render_bbox2D", false);
    Globals::render_tracking_numbers = config.read("render_tracking_numbers", false);

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

void ReadUpperBodyTemplate(string template_path)
{
    // read template from file
    upper_body_template.ReadFromTXT(template_path, 150, 150);

    // resize it to the fixed size that is defined in Config File
    if(upper_body_template.x_size() > Globals::template_size)
    {
        upper_body_template.DownSample(Globals::template_size, Globals::template_size);
    }
    else if(upper_body_template.x_size() < Globals::template_size)
    {
        upper_body_template.UpSample(Globals::template_size, Globals::template_size);
    }
}

void callback(const ImageConstPtr &depth,  const ImageConstPtr &color, const CameraInfoConstPtr &info)
{

    // Get color image
    QImage image_rgb(&color->data[0], color->width, color->height, QImage::Format_RGB888);

    // Get depth
    cv_depth_ptr = cv_bridge::toCvCopy(depth);
    img_depth_ = cv_depth_ptr->image;
    Matrix<double> matrix_depth(info->width, info->height);

    for (int r = 0;r < 480;r++){
        for (int c = 0;c < 640;c++) {
            matrix_depth(c, r) = img_depth_.at<float>(r,c);
        }
    }

    // Generate base camera
    Matrix<double> R = Eye<double>(3);
    Vector<double> t(3, 0.0);
    Vector<double> GP(0.0, 0.99, 0.0, 1.8); // just some placeholders which will be replaced after GP Estimation
    Matrix<double> K(3,3, (double*)&info->K[0]);

    Camera camera(K,R,t,GP);
    PointCloud point_cloud(camera, matrix_depth);


    GP = GPEstimator.ComputeGroundPlane(point_cloud);

    camera = Camera(K,R,t,GP);

    Vector<Vector< double > > detected_bounding_boxes;

    detector->ProcessFrame(camera, matrix_depth, point_cloud, upper_body_template, detected_bounding_boxes);

    strands_perception_people_msgs::UpperBodyDetector detection_msg;
    detection_msg.header = depth->header;

    for(int i = 0; i < detected_bounding_boxes.getSize(); i++)
    {
        detection_msg.pos_x.push_back(detected_bounding_boxes(i)(0));
        detection_msg.pos_y.push_back(detected_bounding_boxes(i)(1));
        detection_msg.width.push_back(detected_bounding_boxes(i)(2));
        detection_msg.height.push_back(detected_bounding_boxes(i)(3));
        detection_msg.dist.push_back(detected_bounding_boxes(i)(4));
        detection_msg.median_depth.push_back(detected_bounding_boxes(i)(5));
    }

    render_bbox_2D(detection_msg, image_rgb, 0, 0, 255, 2);

    sensor_msgs::Image sensor_image;
    sensor_image.header = color->header;
    sensor_image.height = image_rgb.height();
    sensor_image.width  = image_rgb.width();
    vector<unsigned char> image_bits(image_rgb.bits(), image_rgb.bits()+sensor_image.height*sensor_image.width*3);
    sensor_image.data = image_bits;
    sensor_image.encoding = color->encoding;

    // Generate Ground Plane Message
    strands_perception_people_msgs::GroundPlane ground_plane_msg;
    ground_plane_msg.header = depth->header;
    ground_plane_msg.d = GP(3);
    ground_plane_msg.n.push_back(GP(0));
    ground_plane_msg.n.push_back(GP(1));
    ground_plane_msg.n.push_back(GP(2));


    pub_result_image.publish(sensor_image);
    pub_message.publish(detection_msg);
    pub_ground_plane.publish(ground_plane_msg);

}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "upper_body_detector");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    string topic_depth_image;
    string topic_color_image;
    string topic_camera_info;
    string pub_topic_result_image;
    string pub_topic_gp;
    string config_file;
    string template_path;

    string pub_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("config_file", config_file, string(""));
    private_node_handle_.param("template_file", template_path, string(""));
    private_node_handle_.param("depth_image", topic_depth_image, string("/camera/depth/image"));
    private_node_handle_.param("camera_info", topic_camera_info, string("/camera/rgb/camera_info"));
    private_node_handle_.param("color_image", topic_color_image, string("/camera/rgb/image_color"));

    if(strcmp(config_file.c_str(),"") == 0) {
        ROS_ERROR("No config file specified.");
        ROS_ERROR("Run with: rosrun strands_upperbody_detector upper_body_detector _config_file:=/path/to/config");
        exit(0);
    }
    if(strcmp(template_path.c_str(),"") == 0) {
        ROS_ERROR("No template file specified.");
        ROS_ERROR("Run with: rosrun strands_upper_body_detector upper_body_detector _template_path:=/path/to/template");
        exit(0);
    }

    // Create a subscriber.
    message_filters::Subscriber<Image> subscriber_depth(n, topic_depth_image.c_str(), 50);
    message_filters::Subscriber<CameraInfo> subscriber_camera_info(n, topic_camera_info.c_str(), 50);
    message_filters::Subscriber<Image> subscriber_color(n, topic_color_image.c_str(), 50);

    sync_policies::ApproximateTime<Image, Image, CameraInfo> MySyncPolicy(10);
    MySyncPolicy.setAgePenalty(10);

    ReadUpperBodyTemplate(template_path);
    ReadConfigFile(config_file);
    detector = new Detector();


    const sync_policies::ApproximateTime<Image, Image, CameraInfo> MyConstSyncPolicy = MySyncPolicy;

    Synchronizer< sync_policies::ApproximateTime<Image, Image, CameraInfo> > sync(MyConstSyncPolicy,
                                                                                  subscriber_depth, subscriber_color, subscriber_camera_info);

    sync.registerCallback(boost::bind(&callback, _1, _2, _3));


    // Create a topic publisher
    private_node_handle_.param("upperbody_detections", pub_topic, string("/upper_body_detector/detections"));
    pub_message = n.advertise<strands_perception_people_msgs::UpperBodyDetector>(pub_topic.c_str(), 10);

    private_node_handle_.param("upperbody_result_image", pub_topic_result_image, string("/upper_body_detector/image"));
    pub_result_image = n.advertise<sensor_msgs::Image>(pub_topic_result_image.c_str(), 10);

    private_node_handle_.param("ground_plane", pub_topic_gp, string("/ground_plane"));
    pub_ground_plane = n.advertise<strands_perception_people_msgs::GroundPlane>(pub_topic_gp.c_str(), 10);


    ros::spin();

    return 0;
}

