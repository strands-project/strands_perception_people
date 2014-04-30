#pragma once

#include <vector>

// Cannot forward-declare Features as class :/
#include "features.hpp"

namespace cv {
    class Mat;
}

namespace warco {

    void test_covcorr();
    cv::Mat extract_corr(const Features& feats, unsigned x, unsigned y, unsigned w, unsigned h);
    std::vector<cv::Mat> extract_corrs(const Features& feats);

} // namespace warco

