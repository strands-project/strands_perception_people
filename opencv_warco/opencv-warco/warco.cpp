#include "warco.hpp"

#include <fstream>
#include <stdexcept>

#include "covcorr.hpp"
#include "features.hpp"
#include "model.hpp"
#include "to_s.hpp"

#ifndef NDEBUG
#  include <iostream>
#endif

warco::Warco::Patch::Patch(double x, double y, double w, double h, double weight)
    : weight(weight)
    , x(x), y(y), w(w), h(h)
    , model(new PatchModel())
{ }

warco::Warco::Warco(cv::FilterBank fb, const std::vector<warco::Patch>& patches)
    : _fb(fb)
{
    for(auto p : patches)
        _patchmodels.push_back(Patch(p.x, p.y, p.w, p.h));
}

warco::Warco::Warco(std::string name)
{
    this->load(name);
}

warco::Warco::~Warco()
{ }

void warco::Warco::add_sample(const cv::Mat& img, unsigned label)
{
    this->foreach_model(img, [label](const Patch& patch, const cv::Mat& corr) {
        patch.model->add_sample(corr, label);
    });
}

double warco::Warco::train(const std::vector<double>& cvC, std::function<void(unsigned)> progress)
{
    double w_tot = 0.0;
#ifdef _OPENMP
    const unsigned s = _patchmodels.size();
    #pragma omp parallel for reduction(+:w_tot)
    for(unsigned i = 0 ; i < s ; ++i) {
        auto& patch = _patchmodels[i];
#else
    for(auto& patch : _patchmodels) {
#endif
        patch.weight = patch.model->train(cvC);
        w_tot += patch.weight;

        progress(_patchmodels.size() + 1);
    }

    for(auto& patch : _patchmodels) {
        patch.weight /= w_tot;
    }

    progress(_patchmodels.size() + 1);

#ifndef NDEBUG
    if(getenv("WARCO_DEBUG")) {
        // This heuristic only works for default stuff.
        unsigned x = 0, w = static_cast<unsigned>(sqrt(_patchmodels.size()));
        for(const auto& patch : _patchmodels) {
            if(x++ % w == 0)
                std::cout << std::endl;
            std::cout << patch.weight << " ";
        }
    }
#endif

    // Return the average error.
    return w_tot / _patchmodels.size();
}

unsigned warco::Warco::predict(const cv::Mat& img) const
{
    std::vector<double> votes(this->nlbl(), 0.0);

    this->foreach_model(img, [&votes](const Patch& patch, const cv::Mat& corr) {
        unsigned pred = patch.model->predict(corr);

#ifdef _OPENMP
        #pragma omp critical
#endif
        votes[pred] += patch.weight;

#ifndef NDEBUG
        if(getenv("WARCO_DEBUG")) {
            std::cout << " " << pred;
        }
#endif
    });

#ifndef NDEBUG
    if(getenv("WARCO_DEBUG")) {
        std::cout << to_s(votes) << std::endl;
    }
#endif

    // argmax
    return std::max_element(begin(votes), end(votes)) - begin(votes);
}

unsigned warco::Warco::predict_proba(const cv::Mat& img) const
{
    std::vector<double> probas(this->nlbl(), 0.0);

    this->foreach_model(img, [&probas](const Patch& patch, const cv::Mat& corr) {
        auto pred = patch.model->predict_probas(corr);

#ifdef _OPENMP
        #pragma omp critical
#endif
        for(unsigned i = 0 ; i < probas.size() ; ++i)
            probas[i] += pred[i] * patch.weight;

#ifndef NDEBUG
        if(getenv("WARCO_DEBUG")) {
            std::cout << " " << to_s(pred);
        }
#endif
    });

#ifndef NDEBUG
    if(getenv("WARCO_DEBUG")) {
        std::cout << to_s(probas) << std::endl;
    }
#endif

    // argmax
    return std::max_element(begin(probas), end(probas)) - begin(probas);
}

unsigned warco::Warco::nlbl() const
{
    return _patchmodels.front().model->nlbls();
}

void warco::Warco::foreach_model(const cv::Mat& img, std::function<void(const Patch& patch, const cv::Mat& corr)> fn) const
{
    // TODO: take the actual size out of config.
    cv::Mat img50 = img;
    if(img.cols != 50 || img.rows != 50) {
        resize(img, img50, cv::Size(50, 50));
    }

    auto feats = warco::mkfeats(img50, _fb);

#ifdef _OPENMP
    const int s = _patchmodels.size();
    #pragma omp parallel for
    for(int i = 0 ; i < s ; ++i) {
        const auto& p = _patchmodels[i];
#else
    for(const auto& p : _patchmodels) {
#endif
        fn(p, extract_corr(feats, p.x*img50.cols, p.y*img50.rows, p.w*img50.cols, p.h*img50.rows));
    }
}

void warco::Warco::load(std::string name)
{
    _patchmodels.clear();

    _fb.load((name + "/filterbank").c_str());

    std::ifstream f(name + "/warco");
    if(! f)
        throw std::runtime_error("Couldn't load warco file '" + name + "/warco'");

    unsigned n;
    double weight, x, y, w, h;
    f >> n;
    for(unsigned i = 0 ; i < n ; ++i) {
        f >> weight >> x >> y >> w >> h;
        _patchmodels.push_back(Patch(x, y, w, h, weight));
        _patchmodels.back().model->load(name + "/patch" + to_s(i));
    }
}

void warco::Warco::save(std::string name) const
{
    _fb.save((name + "/filterbank").c_str());

    std::ofstream of(name + "/warco");
    if(! of)
        throw std::runtime_error("Couldn't create warco file '" + name + "/warco'");

    unsigned i = 0;
    of << _patchmodels.size() << std::endl;
    for(const auto& p : _patchmodels) {
        of << std::endl << p.weight << " " << p.x << " " << p.y << " " << p.w << " " << p.h;
        p.model->save(name + "/patch" + to_s(i++));
    }
}

