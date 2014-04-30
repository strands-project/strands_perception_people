#include "model.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>

#ifndef NDEBUG
#  include <iostream>
#endif

#include <opencv2/opencv.hpp>

#include "libsvm/svm.h"
#include "dists.hpp"
#include "to_s.hpp"

void warco::test_model()
{
    // TODO - how can I actually test this !?
    // Anyways, already tested in prototype^^
}

warco::PatchModel::PatchModel()
    : _svm(nullptr)
    , _prob(nullptr)
    , _mean(0.0)
{ }

warco::PatchModel::~PatchModel()
{
    this->free_svm();
}

void warco::PatchModel::free_svm()
{
    if(_svm) svm_free_and_destroy_model(&_svm);

    if(_prob) {
        delete[] _prob->x[0];
        delete[] _prob->x;
        delete _prob;
        _prob = nullptr;
    }
}

void warco::PatchModel::add_sample(const cv::Mat& corr, unsigned label)
{
    _corrs.push_back(corr);
    _lbls.push_back(static_cast<double>(label));
}

double warco::PatchModel::train(std::vector<double> C_crossval)
{
    this->free_svm();

    if(C_crossval.empty()) {
        C_crossval.push_back(0.1);
        C_crossval.push_back(1.0);
        C_crossval.push_back(10.);
    }

    // 1. Compute distance matrix
    // 2. train SVM

    auto N = _corrs.size();

    _prob = new svm_problem;
    _prob->l = N;
    _prob->y = &_lbls[0];

    // Construct what will hold the distance/kernel "matrix"
    _prob->x = new svm_node*[N];
    auto* xes = new svm_node[N*(N+2)]; // TODO: Could halve this, but meh!
    for(unsigned i = 0 ; i < N ; ++i) {
        _prob->x[i] = xes + i*(N+2);

        // Make the first entry be the "sample id" as requested in
        // the "precomputed kernel" section of the readme.
        _prob->x[i][0].index = 0;
        _prob->x[i][0].value = 1+i;

        // Make the last of each row be -1 as requested by the API.
        _prob->x[i][N+1].index = -1;
    }

    // Compute the Gram matrix first, but compute the mean in the same run,
    // we'll need it to turn the matrix into a mercer kernel next.
    _mean = 0.0;
    for(unsigned i = 0 ; i < N ; ++i) {
        for(unsigned j = 0 ; j < i ; ++j) {
            double d = dist_cbh(_corrs[i], _corrs[j]);
            _prob->x[i][1+j].index = 1+j;
            _prob->x[j][1+i].index = 1+i;
            _prob->x[i][1+j].value = d;
            _prob->x[j][1+i].value = d;
            _mean += d;
        }
        // The diagonal is outside of above loop to avoid
        // counting it twice (in the mean, mainly).
        double d = dist_cbh(_corrs[i], _corrs[i]);
        _prob->x[i][1+i].index = 1+i;
        _prob->x[i][1+i].value = d;
        _mean += d;
    }

    _mean /= (N*(N+1)/2);

    // Turn it into a mercer kernel next.
    for(unsigned i = 0 ; i < N ; ++i) {
        for(unsigned j = 0 ; j < i ; ++j) {
            double k = std::exp(-_prob->x[i][1+j].value / _mean);
            _prob->x[i][1+j].value = k;
            _prob->x[j][1+i].value = k;
        }
        // Same story about the diagonal here.
        _prob->x[i][1+i].value = std::exp(-_prob->x[i][1+i].value / _mean);
    }

    // Now setup the SVM's parameters to use above kernel.

    svm_parameter param = svm_parameter();
    param.svm_type = C_SVC;
    param.kernel_type = PRECOMPUTED;
    // degree, gamma, coef0 unused. C cross-validated

    // Training settings
    param.cache_size = 1; // MB (for kernels, needs to be >= 0 even if unused.)
    param.eps = 0.001; // "(we usually use 0.00001 in nu-SVC, 0.001 in others)."
    // C_SVC only `nr_weight`, `weight_label` and `weight` unused.
    // NU_SV? only `nu`
    // EPSILON_SVR: `p`
    param.shrinking = int(true);
    param.probability = int(true);

    // *NOTE* Because svm_model contains pointers to svm_problem, you can
    // not free the memory used by svm_problem if you are still using the
    // svm_model produced by svm_train().

    double best = 0.0;
    double best_c = 0.0;
    for(auto c : C_crossval) {
        param.C = c;

        // Checking for correctness of above settings.
        if(const char* err = svm_check_parameter(_prob, &param)) {
            throw std::runtime_error(err);
        }

        std::vector<double> pred(N);
        svm_cross_validation(_prob, &param, 8, &pred[0]);

        // Compute accuracy;
        unsigned N_correct = 0;
        for(unsigned i = 0 ; i < N ; ++i)
            if(pred[i] == _lbls[i])
                ++N_correct;
        double accuracy = N_correct/static_cast<double>(N);

#ifndef NDEBUG
        if(getenv("WARCO_DEBUG")) {
            std::cout << "Cross-validation: C=" << c << " => " << 100.*accuracy << "%" << std::endl;
        }
#endif

        // Keep track of the best C.
        if(accuracy > best) {
            best = accuracy;
            best_c = c;
        }
    }

    // Now train an SVM on the full dataset with the optimal C.
    param.C = best_c;
    _svm = svm_train(_prob, &param);

#ifndef NDEBUG
    if(getenv("WARCO_DEBUG")) {
        std::cout << "#data: " << _prob->l << std::endl;
        std::cout << "#SV: " << _svm->l << " (" << 100.0*_svm->l/_prob->l << "%)" << std::endl;
    }
#endif

    return best;
}

void warco::PatchModel::save(std::string name) const
{
    //Json::Value model;
    //int svm_get_nr_sv(const struct svm_model *model)
    //   This function gives the number of total support vector.
    //void svm_get_sv_indices(const struct svm_model *model, int *sv_indices)
    //   This function outputs indices of support vectors into an array called sv_indices.
    //   The size of sv_indices is the number of support vectors and can be obtained by calling svm_get_nr_sv.
    //   Each sv_indices[i] is in the range of [1, ..., num_traning_data].
    //
    // TODO.
    svm_save_model((name + ".svm").c_str(), _svm);

    std::ofstream of(name + ".model");
    if(! of)
        throw std::runtime_error("Error creating the model file " + name + ".model");

    of << _mean << std::endl;
    of << _corrs.size() << std::endl;
    cv::FileStorage f(name + "corrs.yaml", cv::FileStorage::WRITE);
    for(unsigned i = 0 ; i < _corrs.size() ; ++i) {
        f << "corr" + to_s(i) << _corrs[i];
    }
}

void warco::PatchModel::load(std::string name)
{
    _svm = svm_load_model((name + ".svm").c_str());

    std::ifstream f(name + ".model");
    if(! f)
        throw std::runtime_error("Error opening the model file " + name + ".model");

    f >> _mean;
    unsigned ncorrs = 0;
    f >> ncorrs;
    _corrs.resize(ncorrs);
    cv::FileStorage fs(name + "corrs.yaml", cv::FileStorage::READ);
    for(unsigned i = 0 ; i < ncorrs ; ++i) {
        fs["corr" + to_s(i)] >> _corrs[i];
    }
}

unsigned warco::PatchModel::predict(const cv::Mat& corr) const
{
    if(! _svm)
        throw std::runtime_error("Load model before predicting plx!");

    // We only need to have the kernel evaluation with support vectors.
    // TODO: Not always reallocate, but keep between calls.
#if 0
    // TODO: This doesn't work with saving/loading yet,
    //       simply because we'd need to store each corr's id
    //       in addition and add dummy ones in between.
    auto N = _svm->l;
    svm_node* nodes = new svm_node[N + 2];
    for(int i = 0 ; i < N ; ++i) {
        int iSV = _svm->sv_indices[i];
        // -1 because in the case of a kernel they start at 1!
        double d = dist_cbh(this->_corrs[iSV-1], corr);
        nodes[1+i].index = iSV;
        nodes[1+i].value = std::exp(-d / _mean);
    }
    nodes[0].index = 0;
    nodes[N+1].index = -1;
#else
    auto N = _corrs.size();
    svm_node* nodes = new svm_node[N+2];
    for(unsigned i = 0 ; i < N ; ++i) {
        double d = dist_cbh(this->_corrs[i], corr);
        nodes[1+i].index = 1+i;
        nodes[1+i].value = std::exp(-d / _mean);
    }
    nodes[0].index = 0; // And .value is arbitrary at test-time.
    nodes[N+1].index = -1;
#endif

    unsigned label = static_cast<unsigned>(svm_predict(_svm, nodes));

    delete[] nodes;

    return label;
}

std::vector<double> warco::PatchModel::predict_probas(const cv::Mat& corr) const
{
    // TODO: might want to get that one as an output argument
    //       such that if used in an inner loop doesn't get perma-reallocated.
    std::vector<double> nrvo(svm_get_nr_class(_svm), 0.0);

    // TODO also see comments in predict
    auto N = _corrs.size();
    svm_node* nodes = new svm_node[N+2];
    for(unsigned i = 0 ; i < N ; ++i) {
        double d = dist_cbh(this->_corrs[i], corr);
        nodes[1+i].index = 1+i;
        nodes[1+i].value = std::exp(-d / _mean);
    }
    nodes[0].index = 0; // And .value is arbitrary at test-time.
    nodes[N+1].index = -1;

    svm_predict_probability(_svm, nodes, &nrvo[0]);

    delete[] nodes;

    return nrvo;
}

unsigned warco::PatchModel::nlbls() const
{
    if(!_svm)
        throw std::runtime_error("Calling PatchModel::nlbls before training!");

    return svm_get_nr_class(_svm);
}

