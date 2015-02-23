#pragma once

#include <functional>
#include <string>
#include <vector>

namespace cv {
    class Mat;
}

namespace Json {
    class Value;
}

namespace warco {

    class Patch;

    void foreach_img(const Json::Value& dataset, const char* traintest,
                     std::function<void (unsigned, const cv::Mat&, std::string)> fn);
    Json::Value readJson(const char* filename);
    Json::Value getFilelist(const Json::Value& conf, const char* traintest);
    std::vector<warco::Patch> readPatches(const Json::Value& conf);
    std::vector<double> readCrossvalCs(const Json::Value& conf);
    Json::Value getOrLoadArray(const Json::Value& conf, std::string name);
    Json::Value getOrLoadObject(const Json::Value& conf, std::string name);

} // namespace warco

