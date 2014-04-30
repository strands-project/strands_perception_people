#include "dists.hpp"

#include <stdexcept>
#include <opencv2/opencv.hpp>

#include "cvutils.hpp"

static cv::Mat logp_id(const cv::Mat& m)
{
    return warco::eig_fn(m, [](double lambda) { return log(lambda); });
}

static float euc_sq(const cv::Mat& lA, cv::Mat& lB)
{
    auto d = lB - lA;
    return trace(d*d)[0];
}

float warco::dist_euc(const cv::Mat& corrA, const cv::Mat& corrB)
{
    // TODO: could hoist this, since we'll have duplicate work here.
    cv::Mat lA = logp_id(corrA);
    cv::Mat lB = logp_id(corrB);

    return sqrt(euc_sq(lA, lB));
}

static void test_euc()
{
    using warco::reldiff;

    std::cout << "Euclidean distance... " << std::flush;
    cv::Mat A = warco::randspd(4,4),
            B = warco::randspd(4,4);

    double dAA = warco::dist_euc(A, A);
    if(dAA > 1e-6) {
        std::cerr << "Failed! (d(A,A)=" << dAA << " is not close to 0)" << std::endl;
        throw std::runtime_error("Test assertion failed.");
    }

    double dAB = warco::dist_euc(A, B);
    double dBA = warco::dist_euc(B, A);
    if(reldiff(dAB, dBA) > 1e-6) {
        std::cerr << "Failed! (rel diff (dAB, dBA) = " << reldiff(dAB, dBA) << " is too large)" << std::endl;
        throw std::runtime_error("Test assertion failed.");
    }

    std::cout << "SUCCESS" << std::endl;
}

float warco::dist_cbh(const cv::Mat& corrA, const cv::Mat& corrB)
{
    // TODO: could hoist this, since we'll have duplicate work here.
    cv::Mat lA = logp_id(corrA);
    cv::Mat lB = logp_id(corrB);

    cv::Mat d = lB - lA;
    float E = trace(d*d)[0];

    cv::Mat ab = lA * lB,
            ab2 = ab*ab,
            a2 = lA * lA,   // This and below could be taken out too.
            b2 = lB * lB,   // Or cached.
            a2b2 = a2 * b2;
    float xi = -1./12. * (trace(ab2)[0] - trace(a2b2)[0]);

    return sqrt(E + xi);
}

static void test_cbh()
{
    using warco::reldiff;

    std::cout << "CBH distance... " << std::flush;
    cv::Mat A = warco::randspd(4,4),
            B = warco::randspd(4,4);

    double dAA = warco::dist_cbh(A, A);
    if(dAA > 1e-6) {
        std::cerr << "Failed! (d(A,A)=" << dAA << " is not close to 0)" << std::endl;
        throw std::runtime_error("Test assertion failed.");
    }

    double dAB = warco::dist_cbh(A, B);
    double dBA = warco::dist_cbh(B, A);
    if(reldiff(dAB, dBA) > 1e-6) {
        std::cerr << "Failed! (rel diff (dAB, dBA) = " << reldiff(dAB, dBA) << " is too large)" << std::endl;
        throw std::runtime_error("Test assertion failed.");
    }

    std::cout << "SUCCESS" << std::endl;
}

float warco::dist_geo(const cv::Mat& corrA, const cv::Mat& corrB)
{
    // TODO
    // Weird, from both the paper and logic these should not involve logp_id,
    // but from the code, they do.
    cv::Mat lA = logp_id(corrA);
    cv::Mat lB = logp_id(corrB);

    cv::Mat lA_inv_sqrt = warco::eig_fn(lA, [](double lambda) {
        return 1./sqrt(lambda);
    });

    // I'm sure this `thingy` has a meaning.
    cv::Mat thingy = logp_id(lA_inv_sqrt * lB * lA_inv_sqrt);

    // NOTE: the sqrt is missing in tosato's code in Y_GSVM_Train_deterministic:47
    //       (He doesn't seem to ever execute that part anyways)
    return sqrt(trace(thingy*thingy)[0]);
}

static void test_geo()
{
    using warco::reldiff;

    std::cout << "Geodesic distance... " << std::flush;
    cv::Mat A = warco::randspd(4,4),
            B = warco::randspd(4,4);

    // TODO: Is it not a bug that geodesic is much more sensitive?
    double dAA = warco::dist_geo(A, A);
    if(dAA > 1e-5) {
        std::cerr << "Failed! (d(A,A)=" << dAA << " is not close to 0)" << std::endl;
        throw std::runtime_error("Test assertion failed.");
    }

    // TODO: geo definitely isn't symmetric numerically.
    double dAB = warco::dist_geo(A, B);
    double dBA = warco::dist_geo(B, A);
    if(reldiff(dAB, dBA) > 1e-5) {
        std::cerr << "Failed! (rel diff (dAB, dBA) = " << reldiff(dAB, dBA) << " is too large)" << std::endl;
        throw std::runtime_error("Test assertion failed.");
    }

    std::cout << "SUCCESS" << std::endl;
}

void warco::test_dists()
{
    test_euc();
    test_cbh();
    test_geo();
}

