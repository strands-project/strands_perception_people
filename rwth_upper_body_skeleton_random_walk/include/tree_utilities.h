#ifndef TREE_UTILITIES
#define TREE_UTILITIES

#include <vector>
#include <string>
#include "datatypes.h"
#include <opencv2/opencv.hpp>

class tree
{
private :


    std::vector<float> leaf_nodes;
    std::vector<float> split_nodes;
    std::vector<float> offsets;
    std::vector<float> offsets_weights;
    std::vector<float> vote_length_thresholds;
    std::vector<float> leaf_nodes_orientations;

   // float *leaf_nodes;
   // float *split_nodes;
   // float *offsets;
   // float *offsets_weights;
   // float *leaf_nodes_orientations;

    size_t total_split_nodes;
    size_t total_leaf_nodes;
    size_t total_classes;
    //size_t offset_position;
    size_t total_pixels;
    unsigned short self_no;
    unsigned short image_width, image_height;

    //functions for loading tree model and hyperparameters  from files
    void load_leafnodes(const char * path);
    void load_leafnodes_orientations(const char *path);
    void load_splitnodes(const char * path);
    void load_offsets(const char * path);
    void load_offsetsweights(const char * path);
    void load_vote_length_thresholds(const char *path);

    //function to return pixel vote for each joint && pixel's bodypart label
    void get_pixel_hypothesis(pixel p,\
                              const cv::Mat &depth_image,\
                              float *walk_directions,\
                              float *direction_weight);


    //function to get pixel vote



    //function to get the leaf node reached by the pixel
    int get_pixel_leaf_node(pixel p,const cv::Mat &test_image,float depth);

    //function to compute feature response for a Pixel
    double feature_response(float*F, pixel p,\
                            const cv::Mat &depth_image,\
                            float depth);

    //function to get weighted vote for the pixel
    void GetWeightedVote (float *PointCloud, float *Mode , float ModeWeight,\
                                   float threshold, float *Vote, float &Weight);

    void get_pixel_vote(int leafnode, float * walk_directions, float *direction_weight);



public:


    static std::vector<long> voteforjointcounter;
    static long get_vote_count(unsigned index){return voteforjointcounter[index];}
    static void initialize_vote_count()
    {
        for (unsigned i = 0 ; i < 9 ; i++)
            voteforjointcounter.push_back(0);

    }
    static void re_initialize_vote_count()
    {
        for (unsigned i = 0 ; i < 9 ; i++)
            voteforjointcounter[i] = 0;
    }

    tree (unsigned short width,unsigned short height,unsigned no)
    {   this->total_classes = 14;
        this->image_height = height ;
        this->image_width = width;
        this->self_no = no;
    }
    void load_tree(std::string, unsigned short number); //loading the tree

    void apply_tree(const cv::Mat &depth_image,\
                pixel p,\
                float *walk_directions,\
                float *direction_weight);



};

#endif // TREE_UTILITIES

