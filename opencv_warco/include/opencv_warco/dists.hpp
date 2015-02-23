#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cv {
    class Mat;
}

namespace warco {

    class Distance {
    public:
        typedef std::unique_ptr<Distance> Ptr;

        virtual ~Distance() {};

        virtual bool canprep() const {return false;};
        virtual void prepare(cv::Mat& /*cov*/) const {};
        virtual float operator()(const cv::Mat& corrA, const cv::Mat& corrB) const = 0;

        virtual std::string name() const = 0;

        static Ptr create(std::string name);

    protected:
        Distance() {};
    };

    void test_dists();

} // namespace warco

