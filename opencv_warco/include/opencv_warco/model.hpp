#pragma once

#include <string>
#include <vector>

// For Distance
#include "opencv_warco/dists.hpp"

namespace cv {
    class Mat;
}

struct svm_model;
struct svm_problem;

namespace warco {

    void test_model();

    struct PatchModel {
        PatchModel(std::string distname = "");
        ~PatchModel();

        void add_sample(const cv::Mat& corr, unsigned label);
        bool prepare();
        double train(std::vector<double> C_crossval);
        unsigned predict(cv::Mat& corr) const;
        std::vector<double> predict_probas(cv::Mat& corr) const;

        void max_vars(std::vector<float>& vars) const;
        void normalize_covs(const std::vector<float>& max_stddevs);

        void save(std::string name) const;
        void load(std::string name);

        void save_covs(std::string name) const;
        void save_dists(std::string name) const;
        bool maybe_loaddists(std::string name);

        unsigned nlbls() const;

    protected:
        std::vector<cv::Mat> _corrs;
        std::vector<double> _lbls;
        svm_model* _svm;
        svm_problem* _prob;
        double _mean;
        Distance::Ptr _d;
        std::vector<double> _dists;

        void free_svm();
        void compute_kernel();
    };

} // namespace warco

