#pragma once

#include <string>
#include <vector>

namespace cv {
    class Mat;
}

struct svm_model;
struct svm_problem;

namespace warco {

    void test_model();

    struct PatchModel {
        PatchModel();
        ~PatchModel();

        void add_sample(const cv::Mat& corr, unsigned label);
        double train(std::vector<double> C_crossval);
        unsigned predict(const cv::Mat& corr) const;
        std::vector<double> predict_probas(const cv::Mat& corr) const;

        void save(std::string name) const;
        void load(std::string name);

        unsigned nlbls() const;

    protected:
        std::vector<cv::Mat> _corrs;
        std::vector<double> _lbls;
        svm_model* _svm;
        svm_problem* _prob;
        double _mean;

        void free_svm();
    };

} // namespace warco

