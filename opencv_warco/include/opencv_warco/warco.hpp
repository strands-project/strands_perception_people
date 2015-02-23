#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

// For FilterBank.
// TODO: Maybe keep a unique pointer so fwd decl is enough?
#include "opencv_warco/filterbank.hpp"

namespace cv {
    class Mat;
}

namespace warco {

    struct PatchModel;

    struct Patch {
        double x, y, w, h;
    };

    struct Warco {

        Warco(cv::FilterBank fb, const std::vector<warco::Patch>& patches, std::string distfname);
        Warco(std::string name);
        ~Warco();

        void add_sample(const cv::Mat& img, unsigned label);

        void prepare();
        bool maybe_loaddists(std::string name);

        double train(const std::vector<double>& cv_C, std::function<void()> progress = [](){});
        unsigned predict(const cv::Mat& img) const;
        unsigned predict_proba(const cv::Mat& img) const;

        unsigned nlbl() const;

        void save(std::string name) const;
        void load(std::string name);

        void save_covs(std::string name) const;
        void save_dists(std::string name) const;

    protected:
        struct Patch {
            double weight;
            double x, y, w, h;
            std::unique_ptr<PatchModel> model;

            Patch(double x, double y, double w, double h, std::string distfname, double weight = 0.0);
        };

        std::vector<Patch> _patchmodels;
        std::vector<float> _max_stddevs;

        cv::FilterBank _fb;

        void foreach_model(const cv::Mat& img, std::function<void(const Patch& patch, cv::Mat& corr)> fn) const;
        bool foreach_model(std::function<bool(const PatchModel& model, unsigned i)> fn) const;
        bool foreach_model(std::function<bool(PatchModel& model, unsigned i)> fn);
    };

} // namespace warco

