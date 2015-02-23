#include "opencv_warco/covcorr.hpp"

#include <stdexcept>

#include <opencv2/opencv.hpp>

#include <stdexcept>

#include "opencv_warco/cvutils.hpp"
#include "opencv_warco/features.hpp"

// First, let me try the rude, non-integral-image approach.
// Iv'e got the feeling it will either be as-fast-as or not-
// much-slower-than the II-approach, but use way less memory.
//
// Of course, I might be mistaken.

static cv::Mat extract_raw_cov(const warco::Features& feats, unsigned x, unsigned y, unsigned w, unsigned h)
{
    auto nfeats = feats.size();
    std::vector<float> means(nfeats, 0.0f);

    if(w == 1 && h == 1)
        throw std::runtime_error("Covariance of a single point O.o");

    cv::Rect r(x, y, w, h);

    for(unsigned i = 0 ; i < nfeats ; ++i)
        means[i] = mean(feats[i](r))[0];

    cv::Mat cov(nfeats, nfeats, CV_32FC1, 0.0f);

    // TODO: Use calcCovarMatrix?
    // Doesn't seem to work in our use-case where
    // the different dimensions of samples are spread across cvMats

    // TODO: Could use iterators?
    std::vector<const float*> lines(nfeats);
    for(unsigned iy = 0 ; iy < h ; ++iy) {
        for(unsigned i = 0 ; i < nfeats ; ++i)
            lines[i] = feats[i].ptr<float>(y + iy) + x;

        for(unsigned ix = 0 ; ix < w ; ++ix) {
            for(unsigned i = 0 ; i < nfeats ; ++i) {
                float fi = *lines[i]-means[i];
                cov.at<float>(i,i) += fi*fi;
                for(auto j = i+1 ; j < nfeats ; ++j) {
                    cov.at<float>(i,j) += fi*(*lines[j]-means[j]);
                    cov.at<float>(j,i) += fi*(*lines[j]-means[j]);
                }
            }

            for(auto& line : lines) ++line;
        }
    }

    return cov/(w*h-1);
}

// TODO: Use a unittesting framework.
static void test_cov()
{
    std::cout << "cov... " << std::flush;

    warco::Features fts = {
        (cv::Mat_<float>(3,3) <<
            1.f, 2.f, 3.f,
            4.f, 5.f, 6.f,
            7.f, 8.f, 9.f
        ),
        (cv::Mat_<float>(3,3) <<
            .1f, .2f, .3f,
            .4f, .5f, .6f,
            .7f, .8f, .9f
        ),
    };

    // With a unittest framework, I'd check for this throwing.
    //cv::Mat cov = extract_cov(fts, 0, 0, 1, 1);

    cv::Mat cov = extract_raw_cov(fts, 0, 1, 2, 2);
    warco::assert_mat_almost_eq(cov, (cv::Mat_<float>(2,2) <<
        10./3., 1./3.,
         1./3., .1/3.
    ));

    cov = extract_raw_cov(fts, 2, 1, 1, 2);
    warco::assert_mat_almost_eq(cov, (cv::Mat_<float>(2,2) <<
        4.5, .45,
        .45, .045
    ));

    cov = extract_raw_cov(fts, 0, 0, 3, 3);
    warco::assert_mat_almost_eq(cov, (cv::Mat_<float>(2,2) <<
        7.5, .75,
        .75, .075
    ));

    std::cout << "SUCCESS" << std::endl;
}

static cv::Mat rectify_cov(const cv::Mat& cov)
{
    // "make invertible", "enforce SPDness" ->
    //     Clamp eigenvalues to 1e-4
    //
    //  This is to work around patches where all pixels have the same value
    //  for a certain feature, in which case there'd be a division by 0 in the
    //  correlation computation.
    return warco::eig_fn(cov, [](double l) {
        return std::max(1e-4, l);
    });
}

cv::Mat warco::extract_cov(const Features& feats, unsigned x, unsigned y, unsigned w, unsigned h)
{
    return rectify_cov(extract_raw_cov(feats, x, y, w, h));
}

static cv::Mat cov2corr(const cv::Mat& cov)
{
    std::vector<float> stddev(cov.cols);
    // TODO: Doesn't work. Why?
    //sqrt(nrvo.diag(), stddev);
    auto diag = cov.diag();
    for(unsigned i = 0 ; i < stddev.size() ; ++i)
        stddev[i] = sqrt(std::max(diag.at<float>(i), 1e-4f));

    cv::Mat corr = cov.clone();
    warco::normalize_cov(corr, stddev);
    return corr;
}

void warco::normalize_cov(cv::Mat& cov, const std::vector<float>& max_stddevs)
{
    for(auto y = 0 ; y < cov.rows ; ++y)
        for(auto x = 0 ; x < cov.cols ; ++x)
            cov.at<float>(y,x) /= max_stddevs[x]*max_stddevs[y];
}

static void test_cov2corr()
{
    std::cout << "cov2corr... " << std::flush;

    cv::Mat cov = (cv::Mat_<float>(2,2) <<
         4.f, 20.f,
        20.f, 25.f
    );

    warco::assert_mat_almost_eq(cov2corr(cov), (cv::Mat_<float>(2,2) <<
        1.f, 2.f,
        2.f, 1.f
    ));

    std::cout << "SUCCESS" << std::endl;
}

void warco::test_covcorr()
{
    test_cov();
    test_cov2corr();
}

cv::Mat warco::extract_corr(const Features& feats, unsigned x, unsigned y, unsigned w, unsigned h)
{
    return cov2corr(extract_cov(feats, x, y, w, h));
}

std::vector<cv::Mat> warco::extract_corrs(const Features& feats)
{
    std::vector<cv::Mat> nrvo(25);
    auto i = nrvo.begin();

    if(feats[0].cols != 50 || feats[0].rows != 50)
        throw std::runtime_error("Only works on 50x50 images. Sorry mate.");

    for(auto y = 0 ; y < 5 ; ++y)
        for(auto x = 0 ; x < 5 ; ++x)
            *i++ = cov2corr(extract_cov(feats, 1+8*x, 1+8*y, 16, 16));

    return nrvo;
}

