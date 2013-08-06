#ifndef __fovis_feature_match_hpp__
#define __fovis_feature_match_hpp__

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "keypoint.hpp"

namespace fovis
{

class PyramidLevel;

/**
 * Status of a feature match.
 */
enum MatchStatusCode {
  /**
   * match is ok, but needs depth refinement.
   */
  MATCH_NEEDS_DEPTH_REFINEMENT,
  /**
   * match should be rejected.
   */
  MATCH_REFINEMENT_FAILED,
  /**
   * match is ok.
   */
  MATCH_OK
};

/**
 * \ingroup FovisCore
 * \brief Represents a single image feature matched between two camera images
 * taken at different times.
 *
 * The two frames are referred to as the reference and target frames.
 */
class FeatureMatch
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    /**
     * Initializes a NULL (and useless) feature match.
     */
    FeatureMatch() :
      target_keypoint(NULL),
      ref_keypoint(NULL),
      compatibility_degree(0),
      in_maximal_clique(false),
      inlier(false),
      reprojection_error(0),
      track_id(-1)
    {
    }

    /**
     * Initializes a feature match from a target and reference keypoint.
     */
    FeatureMatch(KeypointData* target_keypoint, KeypointData* ref_keypoint) :
      target_keypoint(target_keypoint),
      ref_keypoint(ref_keypoint),
      compatibility_degree(0),
      in_maximal_clique(false),
      inlier(false),
      reprojection_error(0),
      track_id(-1)
    {
      refined_target_keypoint.copyFrom(*target_keypoint);
    }

    /**
     * The target keypoint.
     */
    KeypointData* target_keypoint;

    /**
     * The reference keypoint.
     */
    KeypointData* ref_keypoint;

    /**
     * The target keypoint, after subpixel refinement of the feature match in
     * image space.
     */
    KeypointData refined_target_keypoint;

    /**
     * binary vector, one entry for every feature match.  Each entry is 1 if
     * the motion according to this match is compatible with the motion
     * according to the other match.
     */
    std::vector<int> consistency_vec;

    /**
     * number of 1s in consistency_vec
     */
    int compatibility_degree;

    /**
     * Is this feature match in the maximal consistency clique
     */
    bool in_maximal_clique;

    /**
     * Is this feature an inlier, used for motion estimation
     */
    bool inlier;

    /**
     * Identifies the match during outlier rejection.
     */
    int id;

    /**
     * The image-space distance between the reference keypoint and the target
     * keypoint reprojected onto the reference image.
     */
    double reprojection_error;

    /**
     * Identifies the feature track externally.  If a feature in a new frame is
     * matched to a feature in the previous frame, then the corresponding
     * FeatureMatch object takes on the track_id value of the FeatureMatch
     * object for the previous frame feature.  If the previous frame feature is
     * new (i.e., wasn't matched to a feature in the previous-previous frame),
     * then the track_id is set to a new value.
     */
    int track_id;

    /**
     * status of the feature match.  Value is one of: \p MATCH_NEEDS_DEPTH_REFINEMENT, \p MATCH_OK, or \p MATCH_REFINEMENT_FAILED.
     */
    MatchStatusCode status;
};

} /*  */

#endif
