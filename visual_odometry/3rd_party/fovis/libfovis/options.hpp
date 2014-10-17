#ifndef __fovis_options_hpp__
#define __fovis_options_hpp__

/** \file options.hpp
 * \brief Options
 *
 * TODO
 */

#include <string>
#include <map>

namespace fovis
{

/**
 * \ingroup FovisCore
 * \typedef VisualOdometryOptions
 * \brief Options
 *
 * VisualOdometryOptions is a key-value dictionary of user-adjustable options that
 * affect the behavior of the visual odometry algorithm.  Keys and values are both
 * expressed as strings
 *
 * Current options are:
 * \verbatim
 *   "feature-window-size"
 *     Type:        Integer
 *     Default:     9
 *     Range:       1+
 *     Description: The size of the n x n image patch surrounding each feature, used for
 *                  keypoint matching.
 *      
 *   "max-pyramid-level"
 *     Type:        Integer
 *     Default:     3
 *     Range:       1+
 *     Description: The maximum Gaussian pyramid level to process the image at.
 *                  Pyramid level 1 corresponds to the original image.
 *
 *   "inlier-max-reprojection-error"
 *     Type:        Double
 *     Default:     1.5
 *     Range:       0+
 *     Description: The maximum image-space reprojection error (in pixels) a
 *                  feature match is allowed to have and still be considered an
 *                  inlier in the set of features used for motion estimation.
 *
 *   "clique-inlier-threshold"
 *     Type:        Double
 *     Default:     0.1
 *     Range:       0+
 *     Description: See Howard's greedy max-clique algorithm for determining the 
 *                  maximum set of mutually consisten feature matches.  This
 *                  specifies the compatibility threshold, in meters.
 *
 *   "min-features-for-estimate"
 *     Type:        Integer
 *     Default:     10
 *     Range:       0+
 *     Description: Minimum number of features in the inlier set for the motion estimate
 *     to be considered valid.
 *
 *   "max-mean-reprojection-error"
 *     Type:        Double
 *     Default:     10.0
 *     Range:       0+
 *     Description: Maximum mean reprojection error over the inlier feature matches for the
 *     motion estimate to be considered valid.
 *
 *   "use-subpixel-refinement"
 *     Type:        Integer
 *     Default:     1
 *     Range:       0 or 1
 *     Description: Specifies whether or not to refine feature matches to
 *     subpixel resolution.
 *
 *   "feature-search-window"
 *     Type:        Integer
 *     Default:     25
 *     Range:       0+
 *     Description: Specifies the size of the search window to apply when
 *     searching for feature matches across time frames.  The search is conducted
 *     around the feature location predicted by the initial rotation estimate.
 *
 *   "target-pixels-per-feature"
 *     Type:        Integer
 *     Default:     250
 *     Range:       1+
 *     Description: Specifies the desired feature density as a ratio of input
 *                  image pixels per feature detected.  This number is used to control
 *                  the adaptive feature thresholding.
 *
 *   "update-target-features-with-refined"
 *     Type:        Integer
 *     Default:     0
 *     Range:       0 or 1
 *     Description: When subpixel refinement is enabled, the refined feature
 *                  locations can be saved over the original feature
 *                  locations.  This has a slightly negative impact on
 *                  frame-to-frame visual odometry, but is likely better when
 *                  using this library as part of a visual SLAM
 *                  algorithm.
 * \endverbatim
 */
typedef std::map<std::string, std::string> VisualOdometryOptions;

}

#endif
