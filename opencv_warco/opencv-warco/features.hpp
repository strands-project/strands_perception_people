#pragma once

#include <vector>

namespace cv {
    class Mat;
    class FilterBank;
}

namespace warco {

    typedef std::vector<cv::Mat> Features;

    Features mkfeats(const cv::Mat& m, const cv::FilterBank& fb);
    void showfeats(const Features& feats);

} // namespace warco

