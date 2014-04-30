#include "mainutils.hpp"

#include <fstream>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include "json/json.h"

#include "warco.hpp"

void warco::foreach_img(const Json::Value& dataset, const char* traintest, std::function<void (unsigned, const cv::Mat&, std::string)> fn)
{
    auto lbls = dataset["classes"];
    for(auto ilbl = lbls.begin() ; ilbl != lbls.end() ; ++ilbl) {
        auto lbl = ilbl.index();
        auto lblname = (*ilbl).asString();

        for(Json::Value fname : dataset[traintest][lblname]) {
            cv::Mat image = cv::imread(fname.asString());
            if(! image.data) {
                std::cerr << "Skipping unreadable image " << fname << std::endl;
                continue;
            }

            fn(lbl, image, fname.asString());
        }
    }
}

Json::Value warco::readDataset(const char* fname)
{
    Json::Value nrvo;

    std::ifstream infile(fname);
    if(! infile)
        throw std::runtime_error("Failed to open configuration file " + std::string(fname));

    Json::Reader reader;
    if(! reader.parse(infile, nrvo))
        throw std::runtime_error(
            "Failed to parse configuration file " + std::string(fname) + ":\n" +
            reader.getFormattedErrorMessages()
        );

    return nrvo;
}

std::vector<warco::Patch> warco::readPatches(const Json::Value& conf)
{
    // Read in the patch definitions from the config file
    // or default if none is defined.
    std::vector<warco::Patch> nrvo;

    if(conf.isMember("patches")) {
        for(const Json::Value& p : conf["patches"]) {
            if(p.size() != 4)
                throw std::runtime_error("One of the patches is uncorrectly specified as it doesn't have four entries.");

            nrvo.push_back(warco::Patch{p[0].asDouble(), p[1].asDouble(), p[2].asDouble(), p[3].asDouble()});
        }
    } else {
        // Default from warco for 50x50
        for(auto y = 0 ; y < 5 ; ++y)
            for(auto x = 0 ; x < 5 ; ++x)
                nrvo.push_back(warco::Patch{(1+8*x)/50., (1+8*y)/50., 16/50., 16/50.});
    }

    return nrvo;
}

std::vector<double> warco::readCrossvalCs(const Json::Value& conf)
{
    std::vector<double> C;

    if(conf.isMember("crossval_C"))
        for(Json::Value c : conf["crossval_C"])
            C.push_back(c.asDouble());
    else
        C = {0.1, 1.0, 10.0};

    return C;
}

