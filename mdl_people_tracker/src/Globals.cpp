#include "Globals.h"
#include <string>

////////////////////////////////////////
// Input paths
////////////////////////////////////////
string Globals::camPath_left = "";
string Globals::sImagePath_left = "";
string Globals::tempDepthL = "";
string Globals::path_to_planes = "";

////////////////////////////////////////
// Distance Range Accepted Detections
////////////////////////////////////////
double Globals::distance_range_accepted_detections;

////////////////////////////////////////
// ROI
////////////////////////////////////////
double Globals::inc_width_ratio;
double Globals::inc_height_ratio;
int Globals::region_size_threshold;

////////////////////////////////////////
// Freespace Parameters
////////////////////////////////////////
double Globals::freespace_scaleZ;
double Globals::freespace_scaleX;
double Globals::freespace_minX;
double Globals::freespace_minZ;
double Globals::freespace_maxX;
double Globals::freespace_maxZ;
double Globals::freespace_threshold;
int Globals::freespace_max_depth_to_cons;

////////////////////////////////////////
// Evaluation Parameters
////////////////////////////////////////
double Globals::evaluation_NMS_threshold;
double Globals::evaluation_NMS_threshold_LM;
double Globals::evaluation_NMS_threshold_Border;
double Globals::evaluation_inc_height_ratio;
int Globals::evaluation_stride;
double Globals::evaluation_scale_stride;
int Globals::evaluation_nr_scales;
int Globals::evaluation_inc_cropped_height;
double Globals::evaluation_greedy_NMS_overlap_threshold;
double Globals::evaluation_greedy_NMS_threshold;

////////////////////////////////////////
// World scale
////////////////////////////////////////
double  Globals::WORLD_SCALE;

////////////////////////////////////////
// height and width for precomputed Segmentations
////////////////////////////////////////
int Globals::dImHeight;
int Globals::dImWidth;

////////////////////////////////////////
// Camera
////////////////////////////////////////
double Globals::baseline;

////////////////////////////////////////
// Number of Frames / offset
////////////////////////////////////////
int Globals::numberFrames;
int Globals::nOffset;

////////////////////////////////////////
// Console output
////////////////////////////////////////
//bool Globals::verbose = true;

////////////////////////////////////////
// Determines if save bounding boxes or not
////////////////////////////////////////
bool Globals::export_bounding_box;
string Globals::bounding_box_path;

////////////////////////////////////////
// Determines if save result images or not
////////////////////////////////////////
bool Globals::export_result_images;
string Globals::result_images_path = " ";

////////////////////////////////////////
// Size of Template
////////////////////////////////////////
int Globals::template_size = 30;


/////////////////////////////////TRACKING PART/////////////////////////

// Detections
double Globals::dSameIdThresh;

bool Globals::cutDetectionsUsingDepth;

// Kalman
int Globals::frameRate;

// Camera
double Globals::farPlane;

//World scale
int Globals::binSize;

// Others
double Globals::pedSizeWVis;

double Globals::pedSizeWCom;
double Globals::pedSizeHCom;

int Globals::history;

//Parameters for image-plane hypothesescom/
double  Globals::dObjHeight;
double  Globals::dObjHVar;

// Colorhistogram
double Globals::cutHeightBBOXforColor;
double Globals::cutWidthBBOXColor;
double Globals::posponeCenterBBOXColor;

// Thresholds for combining the detectiob from left and right camera
double Globals::probHeight;

// Visualisation
bool Globals::render_bbox3D;
bool Globals::render_bbox2D;
bool Globals::render_tracking_numbers;

//MDL parameters for trajectories
double  Globals::k1; // "counterweight": min. support for a hypothesis
double  Globals::k2 ; // rel. importance of #poconst ints vs. poconst double strength
double  Globals::k3; // overlap penalty
double  Globals::k4; // temp. decay for static objects

// Threshold for distinction between static/moving object
double  Globals::minvel;
double  Globals::dMaxPedVel; // This is in meter per second = ~ 5km/h

// Trajectory
double Globals::threshLengthTraj;

// Thresholds for accepted and displayed hypotheses
double  Globals::dTheta2;

// Time ant for temporal decay
double Globals:: dTau;

// Time horizon for event cone search
int  Globals::coneTimeHorizon;
double  Globals::maxHoleLen;
double  Globals::dHolePenalty;

/* Q - the system covariance */
double Globals::sysUncX;
double Globals::sysUncY;
double Globals::sysUncRot;
double Globals::sysUncVel;
double Globals::sysUncAcc;

double Globals::kalmanObsMotionModelthresh;
double Globals::kalmanObsColorModelthresh;


////////////////////////GP Estimator/////////////////////////
int Globals::nrInter_ransac;
int Globals::numberOfPoints_reconAsObstacle;



/////////////////////////// ROI Segmentation /////////////
double Globals::sigmaX;
double Globals::precisionX;
double Globals::sigmaZ;
double Globals::precisionZ;

double Globals::max_height;
double Globals::min_height;


///////////////////////////Recording /////////////////////
bool Globals::from_camera;
string Globals::from_file_path;

//////////////////////////Streaming///////////////////////
string Globals::stream_dest_IP;

////////////////////////HOG Detector////////////////////////
double Globals::hog_max_scale;
double Globals::hog_score_thresh;

