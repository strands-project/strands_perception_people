#ifndef __fovis_grid_filter_hpp__
#define __fovis_grid_filter_hpp__

#include <math.h>
#include <vector>
#include "keypoint.hpp"

namespace fovis
{

/**
 * \brief Places features into grid cells and discards the weakest features in each cell.
 */
class GridKeyPointFilter {
public:
  /**
   * Construct a keypoint filter.
   *
   * Conceptually, lays down a grid over the image and sorts keypoints into a
   * grid cell based on their image locations.  Then removes the weakest
   * keypoints in each cell (using the Keypoint::score field) until there are
   * at most \p max_keypoints_per_bucket in each grid cell.
   *
   * \param img_width the width of the input image.
   * \param img_height the height of the input image.
   * \param bucket_width the width of each bucket.
   * \param bucket_height the height of each bucket.
   * \param max_keypoints_per_bucket the maximum number of keypoints in each
   * grid cell after filtering.
   *
   */
  GridKeyPointFilter (int img_width, int img_height, int bucket_width, 
                      int bucket_height, int max_keypoints_per_bucket) :
      _img_width(img_width), _img_height(img_height),
      _bucket_width(bucket_width), _bucket_height(bucket_height),
      _max_keypoints_per_bucket(max_keypoints_per_bucket),
      _grid_rows(ceil((float)img_height/bucket_height)), 
      _grid_cols(ceil((float)img_width/bucket_width)),
      _buckets(_grid_rows * _grid_cols)
  { 
  }

  /**
   * filters the specified vector of keypoints.
   *
   * \param keypoints input/output parameter.  On output, the weakest keypoints
   * in each grid cell will have been removed from this vector.
   */
  void filter(std::vector<KeyPoint>* keypoints);

private:
  int _img_width;
  int _img_height;
  int _bucket_width;
  int _bucket_height;
  int _max_keypoints_per_bucket; 
  int _grid_rows;
  int _grid_cols;
  std::vector<std::vector<KeyPoint> > _buckets;

};

}

#endif
