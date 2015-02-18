#include "opencv_warco/dists.hpp"

#include <stdexcept>
#include <opencv2/opencv.hpp>

#include "opencv_warco/cvutils.hpp"

// Randomly generated, but computed using original matlab implementation.
static const cv::Mat g_wA = (cv::Mat_<double>(4,4) <<
    4.6503810, -0.8571707,  0.5054186, -0.6182507,
    -0.8571707,  3.5171059,  0.2891554,  1.4221318,
    0.5054186,  0.2891554,  3.6702419, -0.2359455,
    -0.6182507,  1.4221318, -0.2359455,  4.1547181
);
static const cv::Mat g_wB = (cv::Mat_<double>(4,4) <<
    5.8768365,  2.1580387, -2.0379778,  2.1673869,
    2.1580387,  9.7772208, -0.2766553, -1.1106845,
    -2.0379778, -0.2766553,  7.6452496,  0.0017575,
    2.1673869, -1.1106845,  0.0017575, 11.1753981
);

static cv::Mat logp_id(const cv::Mat& m)
{
    return warco::eig_fn(m, [](double lambda) { return log(lambda); });
}

static void test_logp_id()
{
    std::cout << "logp_id... " << std::flush;

    warco::assert_mat_almost_eq(logp_id(g_wA), (cv::Mat_<double>(4,4) <<
        1.5033978, -0.2045823,  0.1300535, -0.1046505,
       -0.2045823,  1.1510779,  0.1123214,  0.3851847,
        0.1300535,  0.1123214,  1.2840035, -0.0731576,
       -0.1046505,  0.3851847, -0.0731576,  1.3458774
    ));

    warco::assert_mat_almost_eq(logp_id(g_wB), (cv::Mat_<double>(4,4) <<
        1.6013708,  0.3237514, -0.3289237,  0.3071247,
        0.3237514,  2.2303997,  0.0127973, -0.1470679,
       -0.3289237,  0.0127973,  1.9869731,  0.0403853,
        0.3071247, -0.1470679,  0.0403853,  2.3704253
    ));

    std::cout << "SUCCESS" << std::endl;
}

static float euc_sq(const cv::Mat& lA, const cv::Mat& lB)
{
    auto d = lB - lA;
    return trace(d*d)[0];
}

class Euclid : public warco::Distance {
public:
    virtual ~Euclid() {}

    virtual std::string name() const { return "euclid"; }
    virtual bool canprep() const { return true; }

    virtual void prepare(cv::Mat& corr) const
    {
        corr = logp_id(corr);
    }

    virtual float operator()(const cv::Mat& lA, const cv::Mat& lB) const
    {
        return sqrt(euc_sq(lA, lB));
    }

    static void test()
    {
        using warco::reldiff;

        std::cout << "Euclidean distance... " << std::flush;
        cv::Mat A = warco::randspd(4,4),
                B = warco::randspd(4,4);

        Euclid d;
        d.prepare(A);
        d.prepare(B);
        auto wA = g_wA.clone(), wB = g_wB.clone();
        d.prepare(wA);
        d.prepare(wB);

        double dAA = d(A, A);
        if(dAA > 1e-6) {
            std::cerr << "Failed! (d(A,A)=" << dAA << " is not close to 0)" << std::endl;
            throw std::runtime_error("Test assertion failed.");
        }

        double dAB = d(A, B);
        double dBA = d(B, A);
        if(reldiff(dAB, dBA) > 1e-6) {
            std::cerr << "Failed! (rel diff (dAB, dBA) = " << reldiff(dAB, dBA) << " is too large)" << std::endl;
            throw std::runtime_error("Test assertion failed.");
        }

        double dwAwB = d(wA, wB);
        if(reldiff(dwAwB, 2.156221) > 1e-6) {
            std::cerr << "Failed! (rel diff (dwAwB=" << dwAwB << ", 2.156221) = " << reldiff(dwAwB, 2.156221) << " is too large)" << std::endl;
            throw std::runtime_error("Test assertion failed.");
        }

        std::cout << "SUCCESS" << std::endl;
    }
};

class Cbh : public warco::Distance {
public:
    virtual ~Cbh() {}

    virtual std::string name() const { return "cbh"; }
    virtual bool canprep() const { return true; }

    virtual void prepare(cv::Mat& corr) const
    {
        corr = logp_id(corr);
    }

    virtual float operator()(const cv::Mat& lA, const cv::Mat& lB) const
    {
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

    static void test()
    {
        using warco::reldiff;

        std::cout << "CBH distance... " << std::flush;
        cv::Mat A = warco::randspd(4,4),
                B = warco::randspd(4,4);

        Cbh d;
        d.prepare(A);
        d.prepare(B);
        auto wA = g_wA.clone(), wB = g_wB.clone();
        d.prepare(wA);
        d.prepare(wB);

        double dAA = d(A, A);
        if(dAA > 1e-6) {
            std::cerr << "Failed! (d(A,A)=" << dAA << " is not close to 0)" << std::endl;
            throw std::runtime_error("Test assertion failed.");
        }

        double dAB = d(A, B);
        double dBA = d(B, A);
        if(reldiff(dAB, dBA) > 1e-6) {
            std::cerr << "Failed! (rel diff (dAB, dBA) = " << reldiff(dAB, dBA) << " is too large)" << std::endl;
            throw std::runtime_error("Test assertion failed.");
        }

        double dwAwB = d(wA, wB);
        if(reldiff(dwAwB, 2.156895) > 1e-6) {
            std::cerr << "Failed! (rel diff (dwAwB=" << dwAwB << ", 2.156895) = " << reldiff(dwAwB, 2.156895) << " is too large)" << std::endl;
            throw std::runtime_error("Test assertion failed.");
        }

        std::cout << "SUCCESS" << std::endl;
    }
};

class Geodesic : public warco::Distance {
public:
    virtual ~Geodesic () {}

    virtual std::string name() const { return "geodesic"; }

    virtual float operator()(const cv::Mat& corrA, const cv::Mat& corrB) const
    {
        // Weird, from both the paper and logic these should not involve logp_id,
        // but from the code, they do. I think the code is wrong.
        // The results are also much worse if we put logp_id here.
        cv::Mat lA = corrA;
        cv::Mat lB = corrB;

        cv::Mat lA_inv_sqrt = warco::eig_fn(lA, [](double lambda) {
            return 1./sqrt(std::max(lambda, 1e-4));
        });

        // I'm sure this `thingy` has a meaning.
        cv::Mat thingy = lA_inv_sqrt * lB * lA_inv_sqrt;

        double d = 0;
        for(double lambda : warco::eigvals(thingy)) {
            double logl = log(std::max(lambda, 1e-4));
            d += logl*logl;
        }

        // NOTE: the sqrt is missing in tosato's code in Y_GSVM_Train_deterministic:47
        //       (He doesn't seem to ever execute that part anyways)
        return sqrt(d);
    }

    static void test()
    {
        using warco::reldiff;

        std::cout << "Geodesic distance... " << std::flush;
        cv::Mat A = warco::randspd(4,4),
                B = warco::randspd(4,4);

        // TODO: Is it not a bug that geodesic is much more sensitive?
        double dAA = Geodesic()(A, A);
        if(dAA > 1e-5) {
            std::cerr << "Failed! (d(A,A)=" << dAA << " is not close to 0)" << std::endl;
            throw std::runtime_error("Test assertion failed.");
        }

        // TODO: geo definitely isn't symmetric numerically.
        double dAB = Geodesic()(A, B);
        double dBA = Geodesic()(B, A);
        if(reldiff(dAB, dBA) > 1e-5) {
            std::cerr << "Failed! (rel diff (dAB, dBA) = " << reldiff(dAB, dBA) << " is too large)" << std::endl;
            throw std::runtime_error("Test assertion failed.");
        }

        double dwAwB = Geodesic()(g_wA, g_wB);
        if(reldiff(dwAwB, 2.1575107) > 1e-6) {
            std::cerr << "Failed! (rel diff (dwAwB=" << dwAwB << ", 2.1575107) = " << reldiff(dwAwB, 2.1575107) << " is too large)" << std::endl;
            throw std::runtime_error("Test assertion failed.");
        }

        std::cout << "SUCCESS" << std::endl;
    }
};

class MyEuclidean : public warco::Distance {
public:
    virtual ~MyEuclidean () {}

    virtual std::string name() const { return "my euclid"; }

    virtual float operator()(const cv::Mat& corrA, const cv::Mat& corrB) const
    {
        return sqrt(euc_sq(corrA, corrB));
    }

    static void test()
    {
        using warco::reldiff;

        std::cout << "[My] Euclidean distance... " << std::flush;
        cv::Mat A = warco::randspd(4,4),
                B = warco::randspd(4,4);

        MyEuclidean d;
        d.prepare(A);
        d.prepare(B);

        double dAA = d(A, A);
        if(dAA > 1e-6) {
            std::cerr << "Failed! (d(A,A)=" << dAA << " is not close to 0)" << std::endl;
            throw std::runtime_error("Test assertion failed.");
        }

        double dAB = d(A, B);
        double dBA = d(B, A);
        if(reldiff(dAB, dBA) > 1e-6) {
            std::cerr << "Failed! (rel diff (dAB, dBA) = " << reldiff(dAB, dBA) << " is too large)" << std::endl;
            throw std::runtime_error("Test assertion failed.");
        }

        std::cout << "SUCCESS" << std::endl;
    }
};

warco::Distance::Ptr warco::Distance::create(std::string name)
{
    if(name == "euclid") {
        return Ptr(new Euclid());
    } else if(name == "cbh") {
        return Ptr(new Cbh());
    } else if(name == "geodesic") {
        return Ptr(new Geodesic());
    } else if(name == "my euclid") {
        return Ptr(new MyEuclidean());
    } else {
        throw std::runtime_error("Unknown distance function: '" + name + "'");
    }
}

void warco::test_dists()
{
    test_logp_id();

    Euclid::test();
    Cbh::test();
    Geodesic::test();

    MyEuclidean::test();
}

