#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace cv {
    class Mat;
}

namespace warco {

    std::vector<double> eigvals(const cv::Mat& m);
    cv::Mat eig_fn(const cv::Mat& m, std::function<double (double)> fn);
    cv::Mat mkspd(cv::Mat m);
    cv::Mat randspd(unsigned rows, unsigned cols);
    void assert_mat_almost_eq(const cv::Mat& actual, const cv::Mat& expected, double reltol = 1e-6);

    inline double reldiff(double a, double b)
    {
        double ref = std::min(std::abs(a), std::abs(b));
        return ref == 0.0 ? 0.0 : std::abs(a - b) / ref;
    }

    void test_cv_utils();

} // namespace warco

