#include "forest_utilities.h"
#include "tree_utilities.h"
#include <iostream>
#include <algorithm>
#include <omp.h>
#include <stdlib.h>
#include "datatypes.h"
#include "mean_shift_utilities.h"



//###################################################################
//////Constructor for intitializing forest parameters////////////////
//###################################################################

FOREST :: FOREST(size_t no_of_tree, unsigned width, unsigned height)
{
    this->image_height = height;
    this->image_width = width;
    this->total_trees = no_of_tree;
    for (size_t i = 0 ; i < this->total_trees ; i++)
    {
            tree tr(this->image_width,this->image_height,i);
            this->trees.push_back(tr);
    }
}

//###################################################################
/////////////////Loading the forest from file///////////////////////
//###################################################################

void FOREST ::load_forest(std::string path)
{
    //tree::initialize_vote_count();
    std::cout << "loading forest....\n";
    for (size_t i = 0; i < this->total_trees ; i++)
    {
            std::cout << "loading tree : " << i+1 << "\n";
            this->trees[i].load_tree(path,i);
    }
    std::cout << "Ready to process frames\n";

}

//Function to apply the forest

void FOREST :: apply_forest(const cv::Mat &depth_image, \
                            pixel p, \
                            float *walk_directions, \
                            float *direction_weight, unsigned short tree_no)
{

        //this->trees[i].total_pixels = N;
        this->trees[tree_no].apply_tree(depth_image, \
                                        p, \
                                        walk_directions, \
                                        direction_weight);
}
