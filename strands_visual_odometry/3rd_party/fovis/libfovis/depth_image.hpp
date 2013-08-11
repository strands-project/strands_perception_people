#ifndef __fovis_depth_image_hpp__
#define __fovis_depth_image_hpp__

#include <inttypes.h>
#include "camera_intrinsics.hpp"
#include "depth_source.hpp"

namespace fovis
{

class KeypointData;

/**
 * \ingroup DepthSources
 * \brief Depth source where a fully dense, metric depth image is available.
 *
 * TODO
 */
class DepthImage : public DepthSource
{
  public:
    DepthImage(const CameraIntrinsicsParameters& rgb_camera_params,
            int depth_width, int depth_height);
    ~DepthImage();

    /**
     * Set the depth image, a width x height array of distances, units given in
     * meters.  Each pixel of the depth image corresponds to a point in the
     * rectified RGB image.
     */
    void setDepthImage(const float* depth_data);

    virtual bool haveXyz(int u, int v) ;
    virtual void getXyz(OdometryFrame * frame);
    virtual void refineXyz(FeatureMatch * matches,
                           int num_matches,
                           OdometryFrame * frame);

    virtual double getBaseline() const { return 0; }

  private:
    int rgbToDepthIndex(double u, double v) const {
      return (int)(v * _y_scale) * _depth_width + (int)(u * _x_scale);
    }
    bool getXyzInterp(KeypointData* kpdata);

    int _rgb_width;
    int _rgb_height;

    int _depth_width;
    int _depth_height;
    float _x_scale;
    float _y_scale;

    Eigen::Matrix<double, 3, Eigen::Dynamic>* _rays;
    float* _depth_data;

    double _fx_inv;
    double _fy_inv;
    double _neg_cx_div_fx;
    double _neg_cy_div_fy;
};

}
#endif
