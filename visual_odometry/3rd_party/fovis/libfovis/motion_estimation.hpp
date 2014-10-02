#ifndef __fovis_motion_estimation_hpp__
#define __fovis_motion_estimation_hpp__

#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "keypoint.hpp"
#include "frame.hpp"
#include "camera_intrinsics.hpp"
#include "feature_match.hpp"
#include "feature_matcher.hpp"
#include "rectification.hpp"
#include "options.hpp"

namespace fovis
{

enum MotionEstimateStatusCode
{
  NO_DATA,
  SUCCESS,
  INSUFFICIENT_INLIERS,
  OPTIMIZATION_FAILURE,
  REPROJECTION_ERROR
};

extern const char* MotionEstimateStatusCodeStrings[];

/**
 * \ingroup FovisCore
 * \brief Does the heavy lifting for frame-to-frame motion estimation.
 *
 * TODO
 */
class MotionEstimator
{
  public:
    MotionEstimator(const Rectification* rectification, const VisualOdometryOptions& options);
    ~MotionEstimator();

    void estimateMotion(OdometryFrame* reference_frame,
                        OdometryFrame* target_frame,
                        DepthSource* depth_source,
                        const Eigen::Isometry3d &init_motion_est,
                        const Eigen::MatrixXd &init_motion_cov);

    bool isMotionEstimateValid() const {
      return _estimate_status == SUCCESS;
    }

    MotionEstimateStatusCode getMotionEstimateStatus() const {
      return _estimate_status;
    }

    const Eigen::Isometry3d& getMotionEstimate() const {
      return *_motion_estimate;
    }

    const Eigen::MatrixXd& getMotionEstimateCov() const {
      return *_motion_estimate_covariance;
    }

    const FeatureMatch* getMatches() const {
      return _matches;
    }

    int getNumMatches() const {
      return _num_matches;
    }

    int getNumInliers() const {
      return _num_inliers;
    }

    int getNumReprojectionFailures() const {
      return _num_reprojection_failures;
    }

    double getMeanInlierReprojectionError() const {
      return _mean_reprojection_error;
    }

    void sanityCheck() const;

  private:
    void matchFeatures(PyramidLevel* ref_level, PyramidLevel* target_level);
    void computeMaximallyConsistentClique();
    void estimateRigidBodyTransform();
    void refineMotionEstimate();
    void computeReprojectionError();

    // convenience variable
    DepthSource* _depth_source;

    FeatureMatcher _matcher;

    // for each feature in the target frame,
    FeatureMatch* _matches;
    int _num_matches;
    int _matches_capacity;

    // total number of feature tracks we've seen
    int _num_tracks;

    // total number of frames processed
    int _num_frames;

    // inlier count
    int _num_inliers;

    // mean inlier feature reprojection error, in pixels.
    double _mean_reprojection_error;

    // how many feature matches failed the reprojection test
    int _num_reprojection_failures;

    const Rectification* _rectification;

    OdometryFrame* _ref_frame;
    OdometryFrame* _target_frame;

    // the motion estimate.
    // Can also be interpreted as the coordinate transformation to bring
    // coordinates in the target frame to the reference frame.
    Eigen::Isometry3d* _motion_estimate;
    // the 6x6 estimate of the covriance [x-y-z, roll-pitch-yaw];
    Eigen::MatrixXd* _motion_estimate_covariance;

    // maximum mean pixel reprojection error across inliers for a motion
    // estimate to be considered valid
    double _max_mean_reprojection_error;

    // maximum pixel reprojection error for a feature match to be
    // considered part of the inlier set
    double _inlier_max_reprojection_error;

    // maximum distance discrepancy for two feature matches to be
    // considered compatible (for inlier clique computation)
    double _clique_inlier_threshold;

    // at least this many features in the inlier set for a motion
    // estimate to be considered valid
    int _min_features_for_valid_motion_estimate;

    // initial motion estimate confidence bound (in pixels)
    // assume that a feature's image space location will not deviate
    // from the location predicted by the initial motion estimate
    // by more than this much
    double _max_feature_motion;

    // refine feature matches to subpixel resolution?
    int _use_subpixel_refinement;

    // after each frame is processed, should we update feature locations in the
    // target frame with the subpixel-refined locations estimated during the
    // featurea matching?
    bool _update_target_features_with_refined;

    MotionEstimateStatusCode _estimate_status;
};

}

#endif
