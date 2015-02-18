#include "opencv_warco/mainutils.hpp"

#include <fstream>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include "opencv_warco/json/json.h"

#include "opencv_warco/warco.hpp"

void warco::foreach_img(const Json::Value& dataset, const char* traintest, std::function<void (unsigned, const cv::Mat&, std::string)> fn)
{
    auto files = getFilelist(dataset, traintest);
    auto lbls = dataset["classes"];

    for(auto ilbl = lbls.begin() ; ilbl != lbls.end() ; ++ilbl) {
        auto lbl = ilbl.index();
        auto lblname = (*ilbl).asString();

        for(Json::Value fname : files[lblname]) {
            cv::Mat image = cv::imread(fname.asString());
            if(! image.data) {
                std::cerr << "Skipping unreadable image " << fname << std::endl;
                continue;
            }

            fn(lbl, image, fname.asString());
        }
    }
}

Json::Value warco::readJson(const char* fname)
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

Json::Value warco::getFilelist(const Json::Value& conf, const char* traintest)
{
    if(conf.isMember(traintest)) {
        return conf[traintest];
    } else {
        return readJson(conf["files"].asCString())[traintest];
    }
}

std::vector<warco::Patch> warco::readPatches(const Json::Value& conf)
{
    // Read in the patch definitions from the config file
    // or default if none is defined.
    std::vector<warco::Patch> nrvo;

    for(const Json::Value& p : getOrLoadArray(conf, "patches")) {
        if(p.size() != 4)
            throw std::runtime_error("One of the patches is uncorrectly specified as it doesn't have four entries.");

        nrvo.push_back(warco::Patch{p[0].asDouble(), p[1].asDouble(), p[2].asDouble(), p[3].asDouble()});
    }

    return nrvo;
}

std::vector<double> warco::readCrossvalCs(const Json::Value& conf)
{
    std::vector<double> C;

    for(Json::Value c : getOrLoadArray(conf, "crossval_C"))
        C.push_back(c.asDouble());

    return C;
}

Json::Value warco::getOrLoadArray(const Json::Value& conf, std::string name)
{
    if(!conf.isMember(name))
        throw std::runtime_error("Value " + name + " not in config!");

    if(conf[name].isArray())
        return conf[name];
    else if(conf[name].isString())
        return readJson(conf[name].asCString());
    else
        throw std::runtime_error("Value " + name + " in config, but it's not an array!");
}

Json::Value warco::getOrLoadObject(const Json::Value& conf, std::string name)
{
    if(!conf.isMember(name))
        throw std::runtime_error("Value " + name + " not in config!");

    if(conf[name].isObject())
        return conf[name];
    else if(conf[name].isString())
        return readJson(conf[name].asCString());
    else
        throw std::runtime_error("Value " + name + " in config, but it's not an array!");
}

