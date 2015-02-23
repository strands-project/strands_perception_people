#pragma once

#include <vector>

// Cannot forward-declare Features as class :/
#include "opencv_warco/features.hpp"

namespace cv {
    class Mat;
}

namespace warco {

    void test_covcorr();
    cv::Mat extract_cov(const Features& feats, unsigned x, unsigned y, unsigned w, unsigned h);
    cv::Mat extract_corr(const Features& feats, unsigned x, unsigned y, unsigned w, unsigned h);
    std::vector<cv::Mat> extract_corrs(const Features& feats);
    void normalize_cov(cv::Mat& cov, const std::vector<float>& max_stddevs);

} // namespace warco

