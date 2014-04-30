#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

// For FilterBank.
// TODO: Maybe keep a unique pointer so fwd decl is enough?
#include "filterbank.hpp"

namespace cv {
    class Mat;
}

namespace warco {

    struct PatchModel;

    struct Patch {
        double x, y, w, h;
    };

    struct Warco {

        Warco(cv::FilterBank fb, const std::vector<warco::Patch>& patches);
        Warco(std::string name);
        ~Warco();

        void add_sample(const cv::Mat& img, unsigned label);

        double train(const std::vector<double>& cv_C, std::function<void(unsigned max)> progress = [](float){});
        unsigned predict(const cv::Mat& img) const;
        unsigned predict_proba(const cv::Mat& img) const;

        unsigned nlbl() const;

        // TODO
        void save(std::string name) const;
        void load(std::string name);

    protected:
        struct Patch {
            double weight;
            double x, y, w, h;
            std::unique_ptr<PatchModel> model;

            Patch(double x, double y, double w, double h, double weight = 0.0);
        };

        std::vector<Patch> _patchmodels;

        cv::FilterBank _fb;

        void foreach_model(const cv::Mat& img, std::function<void(const Patch& patch, const cv::Mat& corr)> fn) const;
    };

} // namespace warco

