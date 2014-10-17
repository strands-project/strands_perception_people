/* 
 * File:   Globals.h
 * Author: dennis
 *
 * Created on May 15, 2009, 10:02 AM
 */

#ifndef _GLOBALS_DENNIS_H
#define	_GLOBALS_DENNIS_H

#include <string>
using namespace std;

class Globals {
public:
    ////////////////////////////////////////
    // Input paths
    ////////////////////////////////////////
    static  string camPath_left;
    static  string sImagePath_left;
    static string tempDepthL;
    static string path_to_planes;

    ////////////////////////////////////////
    // Distance Range Accepted Detections
    ////////////////////////////////////////
    static double distance_range_accepted_detections;

    ////////////////////////////////////////
    // ROI
    ////////////////////////////////////////
    static double inc_width_ratio;
    static double inc_height_ratio;
    static int region_size_threshold;

    ////////////////////////////////////////
    // Freespace Parameters
    ////////////////////////////////////////
    static double freespace_scaleZ;
    static double freespace_scaleX;
    static double freespace_minX;
    static double freespace_minZ;
    static double freespace_maxX;
    static double freespace_maxZ;
    static double freespace_threshold;
    static int freespace_max_depth_to_cons;

    ////////////////////////////////////////
    // Evaluation Parameters
    ////////////////////////////////////////
    static double evaluation_NMS_threshold;
    static double evaluation_NMS_threshold_LM;
    static double evaluation_NMS_threshold_Border;
    static double evaluation_inc_height_ratio;
    static int evaluation_stride;
    static double evaluation_scale_stride;
    static int evaluation_nr_scales;
    static int evaluation_inc_cropped_height;
    static double evaluation_greedy_NMS_overlap_threshold;
    static double evaluation_greedy_NMS_threshold;

    ////////////////////////////////////////
    // World scale
    ////////////////////////////////////////
    static  double  WORLD_SCALE;

    ////////////////////////////////////////
    // height and width for precomputed Segmentations
    ////////////////////////////////////////
    static  int dImHeight;
    static  int dImWidth;

    ////////////////////////////////////////
    // Camera
    ////////////////////////////////////////
    static double baseline;

    ////////////////////////////////////////
    // Number of Frames / offset
    ////////////////////////////////////////
    static  int numberFrames;
    static int nOffset;

    ////////////////////////////////////////
    // Console output
    ////////////////////////////////////////
    static bool verbose;

    ////////////////////////////////////////
    // Determines if save bounding boxes or not
    ////////////////////////////////////////
    static bool export_bounding_box;
    // Path of exported bounding boxes
    static string bounding_box_path;

    ////////////////////////////////////////
    // Determines if save result images or not
    ////////////////////////////////////////
    static bool export_result_images;
    // Path of result images
    static string result_images_path;

    ////////////////////////////////////////
    // Size of Template
    ////////////////////////////////////////
    static int template_size;

/////////////////////////////////TRACKING PART/////////////////////////

    // Detections
    static int frameRate;

    static bool cutDetectionsUsingDepth;

    // Camera
    static  double farPlane;

    //World scale
    static  int    binSize;

    // other
    static  double pedSizeWVis;

    static double pedSizeWCom;
    static double pedSizeHCom;

    static int history;

    static double dSameIdThresh;

    //Parameters for image-plane hypothesescom/
    static  double  dObjHeight;
    static  double  dObjHVar;

    // Colorhistogram
    static  double cutHeightBBOXforColor;
    static  double cutWidthBBOXColor;
    static  double posponeCenterBBOXColor;

    // Thresholds for combining the detectiob from left and right camera
    static  double probHeight;

    // Visualisation
    static  bool render_bbox3D;
    static  bool render_bbox2D;
    static bool render_tracking_numbers;

    //MDL parameters for trajectories
    static  double  k1; // "counterweight": min. support for a hypothesis
    static  double  k2 ; // rel. importance of #poconst ints vs. poconst double strength
    static  double  k3; // overlap penalty
    static  double  k4; // temp. decay for static objects

    // Threshold for distinction between static/moving object
    static  double  minvel;
    static  double  dMaxPedVel; // This is in meter per second = ~ 5km/h

    // Trajectory
    static  double threshLengthTraj;

    // Thresholds for accepted and displayed hypotheses
    static  double  dTheta2;

    // Time ant for temporal decay
    static  double  dTau;

    // Time horizon for event cone search
    static  int  coneTimeHorizon;
    static  double  maxHoleLen;
    static  double  dHolePenalty;

    /* Q - the system covariance */
    static  double sysUncX;
    static  double sysUncY;
    static  double sysUncRot;
    static  double sysUncVel;
    static  double sysUncAcc;

    static double kalmanObsMotionModelthresh;
    static double kalmanObsColorModelthresh;


    /////////////////////////GP estimator//////////////////////
    static int nrInter_ransac;
    static int numberOfPoints_reconAsObstacle;


    /////////////////////////// ROI Segmentation /////////////
    // Blurring parameter
    static double sigmaX;
    static double precisionX;
    static double sigmaZ;
    static double precisionZ;

    static double max_height;
    static double min_height;

    ///////////////////////////Recording /////////////////////
    static bool from_camera;
    static string from_file_path;

    //////////////////////////Streaming///////////////////////
    static string stream_dest_IP;

    ////////////////////////HOG Detector////////////////////////
    static double hog_max_scale;
    static double hog_score_thresh;
};

#endif	/* _GLOBALS_DENNIS_H */
