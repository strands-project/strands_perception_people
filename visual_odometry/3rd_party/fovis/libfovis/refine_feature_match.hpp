#ifndef __fovis_refine_feature_match_hpp__
#define __fovis_refine_feature_match_hpp__

#include "feature_match.hpp"

namespace fovis
{

void refineFeatureMatch(PyramidLevel* ref_level,
                        PyramidLevel* target_level,
                        Eigen::Vector2d ref_uv,
                        Eigen::Vector2d init_target_uv,
                        Eigen::Vector2d * final_target_uv,
                        float *delta_sse);

} /*  */

#endif
