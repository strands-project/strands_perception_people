#include <iostream>

#include "opencv_warco/json/json.h"

#include "opencv_warco/mainutils.hpp"
#include "opencv_warco/warco.hpp"

int main(int argc, char** argv)
{
    if(argc != 3) {
        std::cout << "Usage: " << argv[0] << " CONF_FILE MODEL_NAME" << std::endl;
        std::cout << std::endl;
        std::cout << "CONF_FILE  Path to the JSON config file describing the dataset." << std::endl;
        std::cout << "MODEL_NAME Name of the model which should be loaded. Is a directory." << std::endl;
        return 0;
    }

    std::cout << "Hey there again! Loading the model... " << std::flush;
    Json::Value dataset = warco::readJson(argv[1]);
    warco::Warco model(argv[2]);
    std::cout << "Done." << std::endl;

    std::cout << "Testing" << std::flush;
    std::cerr << "test,predicted,actual" << std::endl;

    Json::Value lbls = dataset["classes"];
    unsigned correct = 0, total = 0;
    warco::foreach_img(dataset, "test", [&model, &lbls, &correct, &total](unsigned lbl, const cv::Mat& image, std::string fname) {
        std::cout << "." << std::flush;

        //unsigned pred = model.predict(image);
        unsigned pred = model.predict_proba(image);

        std::cerr << fname << "," << lbls[pred].asString() << "," << lbls[lbl].asString() << std::endl;

        correct += pred == lbl;
        ++total;
    });

    std::cout << std::endl << "score: " << 100.0*correct/total << "%" << std::endl;

    return 0;
}

