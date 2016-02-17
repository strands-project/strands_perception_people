#include "bodypose_regressor_utilities.h"
#include "forest_utilities.h"
#include <stdlib.h>
#include "mean_shift_utilities.h"
//#include "image_utilities.h"
#include <opencv2/opencv.hpp>
//#include <random>
#include <iostream>
#include <algorithm>
#include <omp.h>
#include "datatypes.h"
#include <time.h>

void compute_3d_position(pixel p, unsigned short focal_length, \
                         unsigned short width, unsigned short height, \
                         float * _3d_position, float depth)
{
    short  cy = height/2;
    short  cx = width/2;

    _3d_position[0] = ((short)p.col - cx)* depth;
    _3d_position[0] = _3d_position[0]/focal_length;

    _3d_position[1] = ((short)p.row - cy)* depth;
    _3d_position[1] = -1*_3d_position[1]/focal_length;

    _3d_position[2] = depth;

    //if (_3d_position[0] !=_3d_position[0] || _3d_position[1] !=_3d_position[1] || _3d_position[2] !=_3d_position[2])
    //    std::cout << "problem";


}

void compute_2d_position(pixel &p, unsigned short focal_length,\
                         unsigned short width, unsigned short height,\
                         float * _3d_position)
{
    short  cy = height/2;
    short  cx = width/2;

    p.col = (focal_length*_3d_position[0])/_3d_position[2];
    p.col = p.col + cx;

    p.row = (focal_length*_3d_position[1])/_3d_position[2];
    p.row = p.row + cy;
    p.row = 480 - p.row;

   // if ((p.row > 480) || (p.col > 640))
   //     std::cout<<"problem";




}

void compute_joint_position(FOREST &f, unsigned short tree_no, pixel p_start, \
                            const cv::Mat &depth_image, std::vector<float> &position, \
                            unsigned short n_steps, float step_size, float median_depth)
{
    float walk_directions[6];
    float direction_weight[2];
    float _3d_position[3];
    float newposition[3];
    pixel p = p_start;
    unsigned short counter = 0;
    position[0] = position[1] = position[2] = 0;

    std::vector<pixel> tmp_p(n_steps);
    std::vector<float> tmpdepths(n_steps);

    for (unsigned short steps = 0; steps < n_steps ; steps++)
    {
            //get the walk direction from pixel p
            f.apply_forest(depth_image,p,walk_directions,direction_weight,tree_no);
            float tmp[3];
            if (direction_weight[0] >  direction_weight[1])
            {
                tmp[0] = walk_directions[0];
                tmp[1] = walk_directions[1];
                tmp[2] = walk_directions[2];
            }
            else
            {
                tmp[0] = walk_directions[3];
                tmp[1] = walk_directions[4];
                tmp[2] = walk_directions[5];
            }

            //get the new pixel based on walk direction and step size
            float depth = depth_image.at<float>(p.row,p.col);
            compute_3d_position(p,530,640,480,_3d_position,depth);
            newposition[0] = step_size*tmp[0] + _3d_position[0];
            newposition[1] = step_size*tmp[1] + _3d_position[1];
            newposition[2] = step_size*tmp[2] + _3d_position[2];
            compute_2d_position(p,530,640,480,newposition);




            if ((depth > (median_depth + 0.5)) || (depth!=depth ) || (depth <= 0) || (p.row > 479) || (p.col > 639)  )
            {
                if (steps < 50)
                {
                     short r = rand()% 11 + 1;
                     r = r - 5;
                     p.row = p_start.row + r;
                     p.col = p_start.col + r;

                }
                else
                    break;
            }
            else
             {

                tmp_p[counter].row = p.row;
                tmp_p[counter].col = p.col;
                tmpdepths[counter] = depth_image.at<float>(p.row,p.col);
                position[0] = position[0] + newposition[0];
                position[1] = position[1] + newposition[1];
                position[2] = position[2] + newposition[2];
                counter++;

            }

    }



    //std::cout  << position[0] << "," << position[1] << "," << position[2] <<"\n";

    position[0] = position[0]/counter;
    position[1] = position[1]/counter;
    position[2] = position[2]/counter;

    //for debugging
    float tmp[3];
    tmp[0] = position[0];
    tmp[1] = position[1];
    tmp[2] = position[2];









}



void compute_upper_body_pose(const cv::Mat &depthimage,
                     FOREST &f,
                     const std::vector<float> &bbox,
                     std::vector<std::vector<float> > &max_scoring_joints)


{

    pixel start_position;
    start_position.col = bbox[0] + (bbox[2]/2);
    start_position.row = bbox[1] + (bbox[3]/2);

    unsigned short steps_per_joint[9] = {60,60,30,1000,1000,1000,1000,1000,1000};
    float step_size_per_joint[9] = {0.05,0.05,0.05,0.1,0.1,0.05,0.05,0.15,0.15};
    unsigned short parents[8] = {0,1,1,1,3,4,5,6};

    for (unsigned short j = 0 ; j < 9 ;j++)
    {
        std::vector<float> position(3);
        compute_joint_position(f,j,start_position,\
                               depthimage,position,steps_per_joint[j],\
                               step_size_per_joint[j],bbox[4]);

        max_scoring_joints[j][0] = position[0];
        max_scoring_joints[j][1] = position[1];
        max_scoring_joints[j][2] = position[2];

        float tmp[3];
        unsigned short parent = j;
        if (j < 8)
             parent = parents[j];

        tmp[0] = max_scoring_joints[parent][0];
        tmp[1] = max_scoring_joints[parent][1];
        tmp[2] = max_scoring_joints[parent][2];

      //  std::cout  << max_scoring_joints[j][0] << "," << max_scoring_joints[j][1] << "," << max_scoring_joints[j][2] <<"\n";

        compute_2d_position(start_position,530,640,480,tmp);



    }

}
