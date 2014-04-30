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
    Json::Value readDataset(const char* filename);
    std::vector<warco::Patch> readPatches(const Json::Value& conf);
    std::vector<double> readCrossvalCs(const Json::Value& conf);

} // namespace warco

