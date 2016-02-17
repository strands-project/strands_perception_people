#ifndef BODYPOSE_REGRESSOR_UTILITIES
#define BODYPOSE_REGRESSOR_UTILITIES
#include<vector>
#include <opencv2/opencv.hpp>
#include "forest_utilities.h"
#include "datatypes.h"

//#####################################################################################
//Wrapper functions for preprocessing input when using ROS
//before applying regression forest to get
//upper body pose
//#####################################################################################

void remove_background(const cv::Mat &depth_image, \
                       std::vector<unsigned short> &bounding_box, \
                       std::vector<float> &depths,  \
                       std::vector<std::vector<float> > &pixel_locations,\
                       std::vector<float> &test_image,\
                       cv::Mat &tmp_depth_image
                      );

void compute_3d_position(pixel p, unsigned short focal_length, \
                         unsigned short width, unsigned short height, \
                         float * _3d_position, float depth);

void compute_2d_position(pixel &p, unsigned short focal_length, \
                         unsigned short width, unsigned short height, \
                         float *_3d_position);

void compute_upper_body_pose(const cv::Mat &depthimage,
                             FOREST &f,
                             const std::vector<float> &bbox,
                             std::vector<std::vector<float> > &max_scoring_joints);

void compute_joint_position(FOREST &f, unsigned short tree_no, pixel p_start, \
                            const cv::Mat &depth_image, std::vector<float> &position, \
                            unsigned short n_steps, float step_size, \
                            float median_depth);

void sample_joint_poistions(const std::vector<std::vector<std::vector<float> > > & joint_positions, \
                            const std::vector<std::vector<std::vector <float > > > &joint_positions_confidence, \
                            std::vector<std::vector<std::vector<float> > > & sampled_joint_postions, \
                            std::vector<std::vector<std::vector <float> > > & sampled_joint_positions_confidence);

void find_top_scoring_joint_postion(const std::vector<float> & joint_scores, \
                                    const std::vector<unsigned short> &joint_id, \
                                    std::vector<unsigned short> & top_scoring_joints,\
                                    unsigned size);




#endif // BODYPOSE_REGRESSOR_UTILITIES

