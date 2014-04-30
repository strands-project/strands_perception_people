#include "cvutils.hpp"

#include <opencv2/opencv.hpp>

cv::Mat warco::eig_fn(const cv::Mat& m, std::function<double (double)> fn)
{
    cv::Mat eigvals, eigvecs;

    if(! eigen(m, eigvals, eigvecs))
        throw std::runtime_error("Cannot eigen-decompose matrix.");

    for(auto eig = eigvals.begin<float>() ; eig != eigvals.end<float>() ; ++eig)
        *eig = fn(*eig);

    // Notice it's the other way around here as in matlab/numpy!
    return eigvecs.t() * cv::Mat::diag(eigvals) * eigvecs;
}

cv::Mat warco::mkspd(cv::Mat m)
{
    completeSymm(m);
    return m + m.rows * cv::Mat::eye(m.rows, m.cols, CV_32F);
}

cv::Mat warco::randspd(unsigned rows, unsigned cols)
{
    cv::Mat m(rows, cols, CV_32F);
    cv::randu(m, 0.f, 1.f);
    return mkspd(m);
}

void warco::assert_mat_almost_eq(const cv::Mat& actual, const cv::Mat& expected, double reltol)
{
    auto maxdiff = norm(actual - expected, cv::NORM_INF);
    auto maxrel = maxdiff / norm(actual, cv::NORM_INF);
    if(maxrel > reltol) {
        std::cerr << "FAILED (abs diff: " << maxdiff << ", rel diff: " << maxrel << ")" << std::endl;
        std::cerr << "Expected:" << std::endl;
        std::cerr << expected << std::endl;
        std::cerr << "actual:" << std::endl;
        std::cerr << actual << std::endl;
        throw std::runtime_error("Test assertion failed.");
    }
}

