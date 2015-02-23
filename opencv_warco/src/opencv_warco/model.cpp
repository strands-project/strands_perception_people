#include "opencv_warco/model.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>

#ifndef NDEBUG
#  include <iostream>
#endif

#include <opencv2/opencv.hpp>

#include "opencv_warco/covcorr.hpp"
#include "opencv_warco/libsvm/svm.h"
#include "opencv_warco/to_s.hpp"

void warco::test_model()
{
    // TODO - how can I actually test this !?
    // Anyways, already tested in prototype^^
}

warco::PatchModel::PatchModel(std::string dname)
    : _svm(nullptr)
    , _prob(nullptr)
    , _mean(0.0)
    , _d(dname.empty() ? nullptr : Distance::create(dname))
    // Note: the above assumes `load` is called ASAP.
{ }

warco::PatchModel::~PatchModel()
{
    this->free_svm();
}

void warco::PatchModel::free_svm()
{
    if(_svm) svm_free_and_destroy_model(&_svm);

    if(_prob) {
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

void warco::PatchModel::max_vars(std::vector<float>& vars) const
{
    if(vars.empty())
        for(int i = 0 ; i < _corrs.front().cols ; ++i)
            vars.push_back(0.0);

    // TODO: test!
    for(auto& corr : _corrs) {
        for(unsigned i = 0 ; i < vars.size() ; ++i)
            vars[i] = std::max(vars[i], corr.at<float>(i,i));
    }
}

void warco::PatchModel::normalize_covs(const std::vector<float>& max_stddevs)
{
    for(auto& corr : _corrs)
        warco::normalize_cov(corr, max_stddevs);
}

bool warco::PatchModel::prepare()
{
    if(_d->canprep()) {
        for(auto& corr : _corrs)
            _d->prepare(corr);
        return true;
    }

    return false;
}

void warco::PatchModel::compute_kernel()
{
    auto N = _corrs.size();

    // But only compute distances if they haven't been preloaded.
    if(_dists.empty()) {
        // Note: it still got the weird format that the first entry of
        // every line has to be its index! Hence N+
        _dists.resize(N*(N+1));

        // Compute the Gram matrix first, but compute the mean in the same run,
        // we'll need it to turn the matrix into a mercer kernel next.
        _mean = 0.0;
        for(unsigned i = 0 ; i < N ; ++i) {
            // According to the readme, the first entry needs to be the
            // datapoint's index, starting at one.
            _dists[i*(N+1)] = i+1;
            for(unsigned j = 0 ; j <= i ; ++j) {
                double d = (*_d)(_corrs[i], _corrs[j]);
                _dists[i*(N+1)+j+1] = _dists[j*(N+1)+i+1] = d;
                _mean += d;
            }
        }

        _mean /= N*(N+1)/2;
    }

    // Turn distances into a mercer kernel.
    for(unsigned i = 0 ; i < N ; ++i) {
        for(unsigned j = 0 ; j <= i ; ++j) {
            double k = std::exp(- _dists[i*(N+1)+j+1] / _mean);
            _dists[i*(N+1)+j+1] = _dists[j*(N+1)+i+1] = k;
        }
    }
}

double warco::PatchModel::train(std::vector<double> C_crossval)
{
    if(C_crossval.empty()) {
        C_crossval.push_back(0.1);
        C_crossval.push_back(1.0);
        C_crossval.push_back(10.);
    }

    this->compute_kernel();

    // Prepare the format libSVM-dense expects.
    auto N = _corrs.size();
    _prob = new svm_problem;
    _prob->l = N;
    _prob->y = &_lbls[0];

    // Each line is a contiguous array.
    _prob->x = new svm_node[N];
    for(unsigned i = 0 ; i < N ; ++i) {
        _prob->x[i].values = &_dists[i*(N+1)];
        _prob->x[i].dim = N;
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

    of << _d->name() << std::endl;
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

    std::string dfname;
    getline(f, dfname);
    _d = Distance::create(dfname);

    f >> _mean;
    unsigned ncorrs = 0;
    f >> ncorrs;
    _corrs.resize(ncorrs);
    cv::FileStorage fs(name + "corrs.yaml", cv::FileStorage::READ);
    for(unsigned i = 0 ; i < ncorrs ; ++i) {
        fs["corr" + to_s(i)] >> _corrs[i];
    }
}

void warco::PatchModel::save_covs(std::string name) const
{
    std::ofstream of(name + ".covs");
    if(! of)
        throw std::runtime_error("Error creating the distance file " + name + ".covs");

    for(auto& corr : _corrs) {
        for(int i = 0 ; i < corr.rows ; ++i) {
            for(int j = i ; j < corr.cols ; ++j) {
                of << corr.at<float>(i,j) << " ";
            }
        }
        of << std::endl;
    }
}

void warco::PatchModel::save_dists(std::string name) const
{
    std::ofstream of(name + ".dists");
    if(! of)
        throw std::runtime_error("Error creating the distance file " + name + ".dists");

    auto N = _corrs.size();

    // Compute the Gram matrix first, but compute the mean in the same run,
    // we'll need it to turn the matrix into a mercer kernel next.
    for(unsigned i = 0 ; i < N ; ++i) {
        for(unsigned j = 0 ; j <= i ; ++j)
            of << (*_d)(_corrs[i], _corrs[j]) << " ";
        for(unsigned j = i+1 ; j < N ; ++j)
            of << 0.0f << " ";
        of << std::endl;
        if(i % 100 == 0) {
            of << std::flush;
            std::cout << "," << std::flush;
        }
    }
}

bool warco::PatchModel::maybe_loaddists(std::string name)
{
    std::ifstream f(name + ".dists");
    if(! f)
        return false;

    double d;
    auto N = _corrs.size();
    _mean = 0.0;
    _dists.resize(N*(N+1));
    // See `compute_kernel` for the details of the +1.

    // Load the Gram matrix and compute the mean in the same run,
    // we'll need it to turn the matrix into a mercer kernel next.
    for(unsigned i = 0 ; i < N ; ++i) {
        // Magic. (See `compute_kernel`)
        _dists[i*(N+1)] = i+1;
        for(unsigned j = 0 ; j <= i ; ++j) {
            f >> d;
            _dists[i*(N+1)+j+1] = _dists[j*(N+1)+i+1] = d;
            _mean += d;
        }
        // Skip all dem zeros.
        for(unsigned j = i+1 ; j < N ; ++j)
            f >> d;
    }

    _mean /= (N*(N+1)/2);

    return !f.fail();
}

unsigned warco::PatchModel::predict(cv::Mat& corr) const
{
    if(! _svm)
        throw std::runtime_error("Load model before predicting plx!");

    _d->prepare(corr);

    auto N = _corrs.size();
    svm_node node = {(int)N, new double[N+1]};
    for(unsigned i = 0 ; i < N ; ++i)
        node.values[i+1] = std::exp(- (*_d)(this->_corrs[i], corr) / _mean);
    // The content of the first value is irrelevant at test-time.

    unsigned label = static_cast<unsigned>(svm_predict(_svm, &node));

    delete[] node.values;

    return label;
}

std::vector<double> warco::PatchModel::predict_probas(cv::Mat& corr) const
{
    // TODO: might want to get that one as an output argument
    //       such that if used in an inner loop doesn't get perma-reallocated.
    std::vector<double> nrvo(svm_get_nr_class(_svm), 0.0);

    _d->prepare(corr);

    auto N = _corrs.size();
    svm_node node = {(int)N, new double[N+1]};
    for(unsigned i = 0 ; i < N ; ++i)
        node.values[i+1] = std::exp(- (*_d)(this->_corrs[i], corr) / _mean);
    // The content of the first value is irrelevant at test-time.

    svm_predict_probability(_svm, &node, &nrvo[0]);

    delete[] node.values;

    return nrvo;
}

unsigned warco::PatchModel::nlbls() const
{
    if(!_svm)
        throw std::runtime_error("Calling PatchModel::nlbls before training!");

    return svm_get_nr_class(_svm);
}

