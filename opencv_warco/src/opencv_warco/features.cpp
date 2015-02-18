#include "opencv_warco/features.hpp"

#include <opencv2/opencv.hpp>
// For CV_BGR2Lab, at least in opencv trunk.
#include <opencv2/imgproc/types_c.h>

#include "opencv_warco/filterbank.hpp"

#ifndef NDEBUG
#  include <iostream>
#  include "opencv_warco/to_s.hpp"
#endif

warco::Features warco::mkfeats(const cv::Mat& m, const cv::FilterBank& fb)
{
    // Layout is (inclusive):
    // 0-2: L, a, b
    // 3: gradient magnitude
    // 4: gradient orientation
    // 5-x: filterbank
    //
    // Where in default warco 0-x is:
    // 5-8: 4 "sharp" DooG gradients
    // 9-12: 4 "smooth" DooG gradients
    Features nrvo(3+2+fb.size());

#ifndef NDEBUG
    if(getenv("WARCO_DEBUG")) {
        std::cout << "m: " << to_s(m) << std::endl;
    }
#endif

    // Get L*a*b* values out of it. They are all in [0,255] range since m is U8.
    cv::Mat lab;
    cvtColor(m, lab, CV_BGR2Lab);
#ifndef NDEBUG
    if(getenv("WARCO_DEBUG")) {
        std::cout << "L*a*b*: " << to_s(lab) << std::endl;
    }
#endif

    // But we work with float matrices only!
    // Might go for signed 16bit at some point, but only as an optimization if
    // needed since we need to be careful with computations.
    cv::Mat labf;
    lab.convertTo(labf, CV_32FC3);
    split(labf, &nrvo[0]);

    const cv::Mat& l = nrvo[0];

#ifndef NDEBUG
    if(getenv("WARCO_DEBUG")) {
        std::cout << "L*: " << to_s(l) << " ; a*: " << to_s(nrvo[1]) << " ; b*: " << to_s(nrvo[2]) << std::endl;
    }
#endif

    // Compute the gradient mag/ori
    cv::Mat dx, dy;
    const int ksize = 1;
    Sobel(l, dx, CV_32F, 1, 0, ksize);
    Sobel(l, dy, CV_32F, 0, 1, ksize);
    magnitude(dx, dy, nrvo[3]);
    phase(dx, dy, nrvo[4]); // in radians [0,2pi] by default.

    // The following makes 30 be the same as 210 degree.
    // Interestingly, the results stay exactly the same.
#if 0
    for(int y = 0 ; y < nrvo[4].rows ; ++y) {
        float* line = nrvo[4].ptr<float>(y);
        for(int x = 0 ; x < nrvo[4].cols ; ++x, ++line)
            if(*line > M_PI)
                *line -= M_PI;
    }
#endif

    // Compute the filterbank.
    fb.filter(l, &nrvo[5]);

    return nrvo;
}

void warco::showfeats(const Features& feats)
{
/*
    for(auto& feat : feats) {
        cv::Mat normalized;
        normalize(feat, normalized, 0.0, 1.0, CV_MINMAX);
        imshow("Feature", normalized);
        cv::waitKey(0);
    }
*/
}

