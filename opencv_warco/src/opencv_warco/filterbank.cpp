#include "opencv_warco/filterbank.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <opencv2/opencv.hpp>

cv::FilterBank::FilterBank()
{ }

cv::FilterBank::FilterBank(const char* fname)
{
    this->load(fname);
}

cv::FilterBank::FilterBank(const FilterBank& other)
    : _kernels(other._kernels)
{ }

cv::FilterBank::~FilterBank()
{ }

void cv::FilterBank::load(const char* fname)
{
    std::ifstream f(fname);
    if(! f)
        throw std::runtime_error("Couldn't open filterbank file " + std::string(fname));

    unsigned n = 0;
    f >> n;

    for(unsigned i = 0 ; i < n ; ++i) {
        unsigned h = 0, w = 0;
        f >> h >> w;

        Mat k(h, w, CV_32FC1);
        for(unsigned y = 0 ; y < h ; ++y) {
            float* line = k.ptr<float>(y);
            for(unsigned x = 0 ; x < w ; ++x)
                f >> *line++;
        }

        if(! f) {
            std::stringstream ss;
            ss << "Something's wrong in filter " << i << " of filterbank " << fname << ": Unexpected end of file.";
            throw std::runtime_error(ss.str());
        }

        this->add_filter(k);
    }
}

void cv::FilterBank::save(const char* fname) const
{
    std::ofstream of(fname);
    if(! of)
        throw std::runtime_error("Couldn't create filterbank file " + std::string(fname));

    of << this->size() << std::endl;

    for(const Mat& k : _kernels) {
        of << k.rows << " " << k.cols << std::endl;
        for(int y = 0 ; y < k.rows ; ++y) {
            const float* line = k.ptr<float>(y);
            for(int x = 0 ; x < k.cols ; ++x)
                of << *line++ << " ";
            of << std::endl;
        }
        of << std::endl;
    }
}

void cv::FilterBank::add_filter(Mat kernel)
{
    _kernels.push_back(kernel);

    // Verify the filter, just to make sure
    if(std::abs(sum(kernel)[0]) > 1e-6)
        std::cerr << "Warning: kernel " << this->size() << " of bank doesn't sum to 0 but to " << sum(kernel)[0] << std::endl;
}

std::size_t cv::FilterBank::size() const
{
    return _kernels.size();
}

void cv::FilterBank::filter(const Mat& in, Mat* out) const
{
    for(const Mat& k : _kernels)
        filter2D(in, *out++, CV_32F, k);
}

std::vector<cv::Mat> cv::FilterBank::filter(const Mat& in) const
{
    std::vector<Mat> nrvo(this->size());
    this->filter(in, &nrvo[0]);
    return nrvo;
}

