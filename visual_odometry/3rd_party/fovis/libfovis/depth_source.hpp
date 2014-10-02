#ifndef __fovis_depth_source_hpp__
#define __fovis_depth_source_hpp__

#include <Eigen/Core>

namespace fovis
{

class OdometryFrame;
class FeatureMatch;

/**
 * \ingroup FovisCore
 * \brief Provides depth estimates for input image pixels.
 *
 * A reliable source of depth estimates at as many pixels in an image as
 * possible is crucial to producing useful visual odometry estimates.  The
 * DepthSource class abstracts out the process of determining depth at each
 * pixel.
 *
 * DepthSource is used in a lazy fashion.  First, keypoints in the input image
 * are identified.  Next, keypoints without depth information are filtered out
 * (pixels where haveXyz() returns false) and discarded.  Later, the actual depth
 * of the remaining keypoints is retrieved using getXyz().  Finally, after
 * features are temporally matched across frames and their keypoint positions
 * refined, DepthSource is used again to refine the depth estimates of the
 * refined keypoint image positions.
 */
class DepthSource
{
  public:
    /**
     * This should return true if it's not certain there's no depth at (u,v).
     * It should be an inexpensive check that is used to avoid pointless (hah!)
     * creation of keypoints. False positives are fine as ling as getXyz gets
     * rid of them.
     */
    virtual bool haveXyz(int u, int v) = 0;

    /**
     * Populate keypoints in frame with XYZ data.
     */
    virtual void getXyz(OdometryFrame * frame) = 0;

    /**
     * Refine XYZ data of target keypoints in matches (usually after
     * subpixel refinement of the matches across time).
     */
    virtual void refineXyz(FeatureMatch * matches,
                           int num_matches,
                           OdometryFrame * frame) = 0;

    /**
     * Return baseline of depth source, if applicable. If not applicable (e.g. for
     * OpenNI devices) return 0.
     */
    virtual double getBaseline() const = 0;

};

}

#endif
