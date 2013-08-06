#include <assert.h>

#include "grid_filter.hpp"

#include <cmath>
#include <algorithm>
#include <iterator>
#include <iostream>

namespace fovis
{

static bool
keypoint_score_comparator(const KeyPoint& a,
                          const KeyPoint& b) {
    return (a.score > b.score);
}

void
GridKeyPointFilter::filter(std::vector<KeyPoint>* keypoints_)
{
  // clear grid buckets - only makes sense if this object is reused
  for(int i=0, n=_buckets.size(); i<n; i++)
    _buckets[i].clear();
  // insert keypoints in corresponding bucket
  for (size_t i=0, num_keypoints=keypoints_->size(); i < num_keypoints; ++i) {
    KeyPoint& kp((*keypoints_)[i]);
    int grid_iy = kp.v / _bucket_height;
    int grid_ix = kp.u / _bucket_width;
    assert(grid_ix >= 0 && grid_ix < _grid_cols);
    assert(grid_iy >= 0 && grid_iy < _grid_rows);
    int bucket_index = grid_iy * _grid_cols + grid_ix;
    _buckets[bucket_index].push_back(kp);
  }
  // clear original keypoints before refilling it with keypoints to keep
  keypoints_->clear();
  // select strongest from each bucket
  for (size_t i=0, num_buckets=_buckets.size(); i < num_buckets; ++i) {
    std::vector<KeyPoint>& bucket_kps(_buckets[i]);
    std::vector<KeyPoint>::iterator new_end_itr;
    if (bucket_kps.size() > static_cast<size_t>(_max_keypoints_per_bucket)) {
      new_end_itr = bucket_kps.begin() + _max_keypoints_per_bucket;
      std::nth_element(bucket_kps.begin(), new_end_itr, bucket_kps.end(), keypoint_score_comparator);
    } else {
      new_end_itr = bucket_kps.end();
    }
    std::copy(bucket_kps.begin(), new_end_itr, std::back_inserter(*keypoints_));
  }
}

} /*  */
