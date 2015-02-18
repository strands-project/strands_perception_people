#include "opencv_warco/cvutils.hpp"

#include <stdexcept>

#include <opencv2/opencv.hpp>

std::vector<double> warco::eigvals(const cv::Mat& m)
{
    std::vector<double> nrvo(m.rows);
    cv::Mat eigvals, eigvecs;

    if(! eigen(m, eigvals, eigvecs))
        throw std::runtime_error("Cannot eigen-decompose matrix.");

    unsigned i = 0;
    switch(m.type()) {
    case CV_32F:
        for(auto eig = eigvals.begin<float>() ; eig != eigvals.end<float>() ; ++eig)
            nrvo[i++] = *eig;
        break;
    case CV_64F:
        for(auto eig = eigvals.begin<double>() ; eig != eigvals.end<double>() ; ++eig)
            nrvo[i++] = *eig;
        break;
    default:
        throw std::runtime_error("eigvals only works for float and double.");
    }

    return nrvo;
}

cv::Mat warco::eig_fn(const cv::Mat& m, std::function<double (double)> fn)
{
    cv::Mat eigvals, eigvecs;

    if(! eigen(m, eigvals, eigvecs))
        throw std::runtime_error("Cannot eigen-decompose matrix.");

    switch(m.type()) {
    case CV_32F:
        for(auto eig = eigvals.begin<float>() ; eig != eigvals.end<float>() ; ++eig)
            *eig = fn(*eig);
        break;
    case CV_64F:
        for(auto eig = eigvals.begin<double>() ; eig != eigvals.end<double>() ; ++eig)
            *eig = fn(*eig);
        break;
    default:
        throw std::runtime_error("eig_fn only works for float and double.");
    }

    // Notice it's the other way around here as in matlab/numpy!
    return eigvecs.t() * cv::Mat::diag(eigvals) * eigvecs;
}

static void test_eig_fn()
{
    std::cout << "eig_fn... " << std::flush;

    // TODO: This is a trivial test only.
    auto m = warco::randspd(4, 4);
    warco::assert_mat_almost_eq(m, warco::eig_fn(m, [](double l) { return l; }));

    std::cout << "SUCCESS" << std::endl;
}

cv::Mat warco::mkspd(cv::Mat m)
{
    completeSymm(m);
    return m + m.rows * cv::Mat::eye(m.rows, m.cols, m.type());
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

void warco::test_cv_utils()
{
    test_eig_fn();
}

