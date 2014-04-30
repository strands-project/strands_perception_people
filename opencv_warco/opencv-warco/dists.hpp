#pragma once

#include <vector>

namespace cv {
    class Mat;
}

namespace warco {

    void test_dists();
    float dist_euc(const cv::Mat& corrA, const cv::Mat& corrB);
    float dist_cbh(const cv::Mat& corrA, const cv::Mat& corrB);
    float dist_geo(const cv::Mat& corrA, const cv::Mat& corrB);

} // namespace warco

