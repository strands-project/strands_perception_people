#ifndef FOREST_UTILITIES
#define FOREST_UTILITIES

#include <string>
#include <vector>
#include "tree_utilities.h"
#include "datatypes.h"
#include <opencv2/opencv.hpp>

class FOREST
{

private :
    size_t total_trees;
    std::vector<tree> trees;
    unsigned short image_width;
    unsigned short image_height;

public:
    //static long get_vote_count(unsigned index) {return tree::get_vote_count(index);}
    FOREST(size_t no_of_tree, unsigned width, unsigned height); //Initializing the forest
    void load_forest(std::string path); //Loading the forest
    void apply_forest(const cv::Mat & depth_image,\
                      pixel p,\
                      float *walk_directions,\
                      float *direction_weight, unsigned short tree_no); //Applying the forest
};

#endif // FOREST_UTILITIES

