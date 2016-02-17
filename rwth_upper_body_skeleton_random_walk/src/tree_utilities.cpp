#include "tree_utilities.h"
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <opencv2/opencv.hpp>

//###################################################################
////////////Function to load tree from file//////////////////////////
//###################################################################

void tree::load_tree(std::string path, unsigned short number)
{

    char fullpath[200];
    //loading the leaf nodes
    //loading the split nodes
    fullpath[0] = '\0';
    sprintf(fullpath,"%sSplitNodes_tree_%d",path.c_str(),number+1);
    load_splitnodes(fullpath);

    //loading the offsets
    fullpath[0] = '\0';
    sprintf(fullpath,"%sModes_tree_%d",path.c_str(),number+1);
    load_offsets(fullpath);

    //loading the offsetsweights
    fullpath[0] = '\0';
    sprintf(fullpath,"%sModeWeights_tree_%d",path.c_str(),number+1);
    load_offsetsweights(fullpath);


}

//###################################################################
////////////////////////Loading leaf nodes///////////////////////////
//###################################################################

void tree ::load_leafnodes(const char *path)
{
    std::ifstream file(path,std::ios::in | std::ios::binary);
    if (!file)
    {
        std::cout << "Please provide a valid path\n";
         return;
    }
    else
    {
        std::cout << "....loading leafnodes ...\n";
        int size ;
        file.read((char *) &size , sizeof size);
        int tmp_data;
        this->leaf_nodes.resize(size);
        //this->leaf_nodes = (float *)malloc(size * sizeof(float));
        for (int i = 0 ; i < size ; i++)
        {
            file.read((char *) &tmp_data , sizeof tmp_data);
            this->leaf_nodes[i] = tmp_data;
        }

        //computing the total leaf nodes in the tree
        //unsigned long tmp = this->leaf_nodes.size();
        this->total_leaf_nodes = size/this->total_classes;
    }

}

//########################################################################
//////////////loading leaf nodes orientations////////////////////////////
//#######################################################################

void tree::load_leafnodes_orientations(const char *path)
{
    std::ifstream file(path,std::ios::in|std::ios::binary);
    if(!file)
    {
        std::cout << "file not found\n";
    }
    int size;
    file.read((char *) &size, sizeof size);
    int tmp_data;
    this->leaf_nodes_orientations.resize(size);
    //this->leaf_nodes_orientations = (float *)malloc(size * sizeof(float));
    for (int i = 0; i < size; i++)
    {
        file.read((char *)&tmp_data,sizeof tmp_data);
        this->leaf_nodes_orientations[i] = tmp_data;
    }
}

//###################################################################
////////////////////Loading split nodes/////////////////////////////
//###################################################################

void tree ::load_splitnodes(const char *path)
{
    std::ifstream file(path,std::ios::in | std::ios::binary);
    if (!file)
    {
        std::cout << "Please provide a valid path";
         return;
    }
    else
    {
        std::cout << "....loading Split Nodes ....\n";
        int size ;
        file.read((char *) &size , sizeof size);
        int tmp_data;
        this->split_nodes.resize(size);
       //this->split_nodes = (float *)malloc(size * sizeof(float));
        for (int i = 0 ; i < size ; i++)
        {
            file.read((char *) &tmp_data , sizeof tmp_data);
            this->split_nodes[i] = tmp_data;
        }

        //computing the total split nodes in the tree
        //unsigned long tmp = this->split_nodes.size();
        this->total_split_nodes = size/5;
        }
}


//###################################################################
/////////////////////////Loading offsets////////////////////////////
//###################################################################

void tree ::load_offsets(const char *path)
{
    std::ifstream file(path,std::ios::in | std::ios::binary);
    if (!file)
    {
        std::cout << "Please provide a valid path";
        return;
    }
    else
    {
        std::cout << "....loading offsets....\n";
        int size;
        file.read((char *) &size , sizeof size);
        int tmp_data;
        this->offsets.resize(size);
        this->total_leaf_nodes = size/6;
        //this->offsets = (float *)malloc(size * sizeof(float));
        for (int i = 0 ; i < size ; i++)
        {
            file.read((char *) &tmp_data , sizeof tmp_data);
            this->offsets[i] = tmp_data;
        }
    }
}


//###################################################################
////////////////////Loading offsets weights//////////////////////////
//###################################################################

void tree :: load_offsetsweights(const char *path)
{
        std::ifstream file(path,std::ios::in | std::ios::binary);
        if (!file)
        {
            std::cout << "Please provide a valid path";
            return;
        }
        else
        {
            std::cout << "....loading offset weights....\n";
            int size ;
            file.read((char *) &size , sizeof size);
            int tmp_data;
            this->offsets_weights.resize(size);
          // this->offsets_weights = (float *)malloc(size * sizeof(float));
            for (int i = 0 ; i < size ; i++)
            {
                file.read((char *) &tmp_data , sizeof tmp_data);
                this->offsets_weights[i] = tmp_data;
            }
        }

}

//###################################################################
//////////////////Loading vote length thresholds/////////////////////
//###################################################################

void tree ::load_vote_length_thresholds(const char *path)
{
    std::ifstream file(path);
    std::string line;
    std::cout << "loading thresholds ...\n";
    while (std::getline(file, line))
    {
        this->vote_length_thresholds.push_back(::atof(line.c_str()));
    }

}

//###################################################################
////////////computing feature response for a pixel///////////////////
//###################################################################

double tree ::feature_response(float *F, pixel p, \
                               const cv::Mat &depth_image,\
                               float depth)
{

    double response;
    long linearindex;
    int tmpx = round(((F[0]*1.5)/depth + p.row));
    int tmpy = round(((F[1]*1.5)/depth + p.col));


    //=================for debugging======================
    //std::cout << tmpx << ":" << tmpy << "\n";

    if ( (tmpx > this->image_height) || (tmpx < 1) )
        tmpx = 0;
    if ( (tmpy > this->image_width) || (tmpy < 1) )
        tmpy = 0;

    if ( (tmpx == 0 ) || (tmpy == 0 ))
        {
        //depth1 =  (unsigned long)( depths[i] *1000);
            response = abs(100000 -  (unsigned long)( depth *1000));

        }
   else
        {
            //linearindex = this->image_height*(tmpy - 1)+ tmpx;
            double offset_depth = depth_image.at<float>(tmpx,tmpy);

            if (abs(offset_depth) == 0)
                response = abs(100000 - (unsigned long)(depth*1000));
            else
                response = abs((unsigned long)(offset_depth*1000) -(unsigned long)( depth*1000));
        }
    return response;
}

//###################################################################
//////////computing vote from a pixel for a joint////////////////////
//###################################################################

void tree ::GetWeightedVote(float *PointCloud, \
                            float *Mode, float ModeWeight, \
                            float threshold, \
                            float *Vote, float &Weight)
{

    float distance = Mode[0]*Mode[0] + Mode[1]*Mode[1] + Mode[2]*Mode[2];
    distance = sqrt(distance);

    //debugging
    //std::cout << "\n";
    //std::cout << "Mode : " << Mode[0] << "," << Mode[1] << "," << Mode[2] << "\n";
    //std::cout << "PointCloud : " << PointCloud[0] << "," << PointCloud[1] << "," << PointCloud[2] <<"\n";
    //std::cout << "Distance : " << distance << "\n";
    //std::cout << "threshold : " << threshold <<"\n";

    Weight = 0;
    Vote[0] = 0;
    Vote[1] = 0;
    Vote[2] = 0;

   //std :: cout << "\nPoint :" << PointCloud[0] << "\t" << PointCloud[1] << "t" << PointCloud[2];
   //std :: cout << "\n";

    if (distance > threshold)
        return;
    else
    {
        Vote[0] = PointCloud[0] + Mode[0];
        Vote[1] = PointCloud[1] + Mode[1];
        Vote[2] = PointCloud[2] + Mode[2];
        Weight  = PointCloud[2]*PointCloud[2]*ModeWeight;
        if (Vote[0] + Vote[1] + Vote[2] == 0)
            Weight = 0;
        //std :: cout << "\nModeWeight:" << ModeWeight <<"\n";
        //std :: cout << "\nWeight:" << Weight <<"\n";
    }

}

//###################################################################
////////computing vote from a pixel for each joint of interest///////
//###################################################################

void tree::get_pixel_vote(int leafnode,float * walk_directions,float *direction_weight)
{
    if (leafnode > this->total_leaf_nodes || leafnode < 0)
        return;

    walk_directions[0] = this->offsets[leafnode]/100;
    walk_directions[1] = this->offsets[leafnode + this->total_leaf_nodes]/100;
    walk_directions[2] = this->offsets[leafnode + 2*this->total_leaf_nodes]/100;
    direction_weight[0] = this->offsets_weights[leafnode]/100;

    walk_directions[3] = this->offsets[3*this->total_leaf_nodes + leafnode]/100;
    walk_directions[4] = this->offsets[4*this->total_leaf_nodes + leafnode]/100;
    walk_directions[5] = this->offsets[5*this->total_leaf_nodes + leafnode]/100;
    direction_weight[1] = this->offsets_weights[this->total_leaf_nodes + leafnode]/100;




}

//#############################jointscounter[joint]######################################
////push the pixel through the tree  until it reaches a leaf/////////
//###################################################################

int tree :: get_pixel_leaf_node(pixel p, const cv::Mat &test_image, float depth)
{

    //=====================for debugging========================
   // std::cout << "\n" << p.row << ":" << p.col << ":" << depth <<"\n";

    int currentnode = 0;
    int type = 1;
    double response;
    float F[2];


    while (type == 1)
    {
        F[0] =  this->split_nodes[currentnode + 2*this->total_split_nodes]/100;
        F[1] =  this->split_nodes[currentnode + 3*this->total_split_nodes]/100;
        response = feature_response(F,p,test_image,depth);
        if (response <= (this->split_nodes[currentnode + 4*this->total_split_nodes]/100))
            currentnode  =(int)this->split_nodes[currentnode]/100;
        else
            currentnode  = (int)this->split_nodes[currentnode + this->total_split_nodes]/100;
        if (currentnode > 0)
            type = 1;
        else
            type = -1;
        //====================for debugging =================
       // std::cout << currentnode << "\n";

        currentnode = abs(currentnode) - 1;
    }


    return (currentnode + 1) ;


}

//#########################################################################
//################ computing overall response from a pixel i.e ############
//################  votes to all joints of interest     ###################
//###############      body part it belongs to         ####################
//#########################################################################

void tree :: get_pixel_hypothesis(pixel p, \
                                  const cv::Mat &depth_image, \
                                  float *walk_directions, \
                                  float *direction_weight)
{


    float depth = depth_image.at<float>(p.row,p.col);
    int LeafNode = get_pixel_leaf_node(p,depth_image,depth);

    //std::cout << "\n " << LeafNode << "\n";

    int index = LeafNode -1;
    get_pixel_vote(index,walk_directions,direction_weight);

}

//###################################################################
//////applying the tree model to all the pixels in test image////////
//###################################################################

void tree :: apply_tree(const cv::Mat &depth_image, \
                        pixel p, \
                        float *walk_directions, \
                        float *direction_weight)
{


        get_pixel_hypothesis(p,depth_image,walk_directions,direction_weight);


}






















