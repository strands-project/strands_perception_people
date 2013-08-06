#ifndef __fovis_visual_odometry_hpp__
#define __fovis_visual_odometry_hpp__

#include <stdint.h>

#include <Eigen/Geometry>

#include "keypoint.hpp"
#include "camera_intrinsics.hpp"
#include "frame.hpp"
#include "depth_source.hpp"
#include "motion_estimation.hpp"
#include "options.hpp"

namespace fovis
{

/**
 * Utility class so that the VisualOdometry class not need
 * EIGEN_MAKE_ALIGNED_OPERATOR_NEW.
 */
class VisualOdometryPriv
{
  private:
    friend class VisualOdometry;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // best estimate for current position and orientation
    Eigen::Isometry3d pose;

    // transformation relating reference frame and most recent frame
    Eigen::Isometry3d ref_to_prev_frame;

    // best estimate of motion from current to previous frame
    Eigen::Isometry3d motion_estimate;
    // the 6x6 estimate of the covriance [x-y-z, roll-pitch-yaw];
    Eigen::MatrixXd motion_estimate_covariance;

    Eigen::Matrix3d initial_homography_est;
    Eigen::Isometry3d initial_motion_estimate;
    Eigen::MatrixXd initial_motion_cov;
};

/**
 * \ingroup FovisCore
 * \brief Main visual odometry class.
 * \code
 * #include <fovis/fovis.hpp>
 * \endcode
 *
 * This is the primary fovis class for estimating visual odometry.
 *  To use it, you'll need three things:
 * \li a source of grayscale input images.
 * \li a \ref DepthSource that can estimate the distance to as many pixels in the input images as possible.
 * \li a \ref Rectification object for converting the source image coordinates
 * to a rectified pinhole projection coordinate system.  This is typically used
 * to correct radial lens distortion.
 *
 * A typical use case for the VisualOdometry class is to repeatedly call
 * processFrame() as new image data is available, which estimates the camera
 * motion and makes the resulting estimation data available via accessor
 * methods.
 *
 * Options to control the behavior of the visual odometry algorithm can be
 * passed in to the constructor using a \ref VisualOdometryOptions object.
 */
class VisualOdometry
{
  public:
    /**
     * Constructs a new visual odometry estimator.
     *
     * \param rectification specifies the input image dimensions, as well as the
     * mapping from input image coordinates to rectified image coordinates.
     * \param options controls the behavior of the estimation algorithms.  This
     * is specified as a key/value dictionary.
     */
    VisualOdometry(const Rectification* rectification,
                   const VisualOdometryOptions& options);

    ~VisualOdometry();

    /**
     * process an input image and estimate the 3D camera motion between \p gray
     * and the frame previously passed to this method.  The estimated motion
     * for the very first frame will always be the identity transform.
     *
     * \param gray a new input image.  The image dimensions must match those
     * passed in to the constructor, and the image data must be stored in
     * row-major order, with no pad bytes between rows.  An internal copy of
     * the image is made, and the input data is no longer needed once this
     * method returns.
     * \param depth_source a source of depth information that can either
     * provide a depth estimate at each pixel of the input image, or report
     * that no depth estimate is available.
     */
    void processFrame(const uint8_t* gray, DepthSource* depth_source);

    /**
     * Retrieves the integrated pose estimate.  On initialization, the camera
     * is positioned at the origin, with +Z pointing along the camera look
     * vector, +X to the right, and +Y down.
     */
    const Eigen::Isometry3d& getPose() {
      return _p->pose;
    }

    /**
     * Retrieve the current reference frame used for motion estimation.  The
     * reference frame will not change as long as new input frames are easily
     * matched to it.
     */
    const OdometryFrame* getReferenceFrame() const {
      return _ref_frame;
    }

    /**
     * Retrieve the current target frame used for motion estimation.
     */
    const OdometryFrame* getTargetFrame() const {
      return _cur_frame;
    }

    /**
     * If this returns true, then the current target frame will become the
     * reference frame on the next call to processFrame().
     */
    bool getChangeReferenceFrames() const {
      return _change_reference_frames;
    }

    /**
     * \return whether motion estimation succeeded on the most recent call to
     * processFrame(), or provides a rough failure reason.
     */
    MotionEstimateStatusCode getMotionEstimateStatus() const {
      return _estimator->getMotionEstimateStatus();
    }

    /**
     * \return the estimated camera motion from the previous frame to the
     * current frame.
     */
    const Eigen::Isometry3d& getMotionEstimate() const {
      return _p->motion_estimate;
    }

    /**
     * \return the covariance matrix resulting from the final nonlinear
     * least-squares motion estimation step.
     */
    const Eigen::MatrixXd& getMotionEstimateCov() const {
      return _p->motion_estimate_covariance;
    }

    /**
     * \return the \ref MotionEstimator object used internally.
     */
    const MotionEstimator* getMotionEstimator() const {
      return _estimator;
    }

    /**
     * \return the threshold used by the FAST feature detector.
     */
    int getFastThreshold() const {
      return _fast_threshold;
    }

    /**
     * \return the 2D homography computed during initial rotation estimation.
     */
    const Eigen::Matrix3d & getInitialHomography() const {
      return _p->initial_homography_est;
    }

    /**
     * \return the options passed in to the constructor.
     */
    const VisualOdometryOptions& getOptions() const {
      return _options;
    }

    /**
     * \return a reasonable set of default options that can be passed in to the
     * constructor if you don't know or care about the options.
     */
    static VisualOdometryOptions getDefaultOptions();

    /**
     * Performs some internal sanity checks and aborts the program on failure.
     * This is for debugging only.
     */
    void sanityCheck() const;

  private:
    void prepareFrame(OdometryFrame* frame);

    Eigen::Quaterniond estimateInitialRotation(const OdometryFrame* prev,
                                               const OdometryFrame* cur,
                                               const Eigen::Isometry3d
                                               &init_motion_estimate =
                                               Eigen::Isometry3d::Identity());

    const Rectification* _rectification;

    OdometryFrame* _ref_frame;
    OdometryFrame* _prev_frame;
    OdometryFrame* _cur_frame;

    MotionEstimator* _estimator;

    VisualOdometryPriv* _p;

    bool _change_reference_frames;

    long _frame_count;

    // === tuning parameters ===

    int _feature_window_size;

    int _num_pyramid_levels;

    // initial feature detector threshold
    int _fast_threshold;

    // params for adaptive feature detector threshold
    int _fast_threshold_min;
    int _fast_threshold_max;
    int _target_pixels_per_feature;
    float _fast_threshold_adaptive_gain;

    bool _use_adaptive_threshold;
    bool _use_homography_initialization;

    // if there are least this many inliers in the previous motion estimate,
    // don't change reference frames.
    int _ref_frame_change_threshold;

    // Which level of the image pyramid to use for initial rotation estimation
    int _initial_rotation_pyramid_level;

    VisualOdometryOptions _options;
};

}
#endif
