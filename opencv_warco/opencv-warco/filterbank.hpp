#pragma once

#include <vector>

namespace cv {
    class Mat;

    class FilterBank {
    public:
        FilterBank();
        FilterBank(const char* fname);
        // The following two are defined explicitly
        // just to avoid including OpenCV here.
        FilterBank(const FilterBank& other);
        ~FilterBank();

        void load(const char* fname);
        void save(const char* fname) const;

        void add_filter(Mat kernel);
        std::size_t size() const;

        void filter(const Mat& in, Mat* out_begin) const;
        std::vector<Mat> filter(const Mat& in) const;

    protected:
        std::vector<Mat> _kernels;
    };

} // namespace cv

