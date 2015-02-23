#include <iostream>
#include <string>

#include "opencv_warco/json/json.h"

#include "opencv_warco/dists.hpp"
#include "opencv_warco/filterbank.hpp"
#include "opencv_warco/mainutils.hpp"
#include "opencv_warco/warco.hpp"

int main(int argc, char** argv)
{
    if(argc < 3 || 4 < argc) {
        std::cout << "Usage: " << argv[0] << " CONF_FILE MODEL_NAME [DIST_DIR]" << std::endl;
        std::cout << std::endl;
        std::cout << "CONF_FILE  Path to the JSON config file describing the dataset." << std::endl;
        std::cout << "MODEL_NAME Name of the model which should be save. Is a directory." << std::endl;
        std::cout << "DIST_DIR   Directory holding precomputed distances." << std::endl;
        return 0;
    }

    Json::Value dataset = warco::readJson(argv[1]);
    auto patches = warco::readPatches(dataset);
    auto fb = cv::FilterBank(dataset["filterbank"].asCString());
    auto dfn = dataset.get("dist", "cbh").asString();
    bool cov = dataset.get("cov", "true").asBool();
    auto C = warco::readCrossvalCs(dataset);
    warco::Warco model(fb, patches, dfn);

    std::cout << "Loading images... " << std::flush;
    warco::foreach_img(dataset, "train", [&model](unsigned lbl, const cv::Mat& image, std::string) {
        model.add_sample(image, lbl);
    });
    std::cout << "Done" << std::endl;

    if(argc == 4) {
        std::cout << "Checking for cached distances... " << std::flush;
        if(model.maybe_loaddists(argv[3]))
            std::cout << "Found some" << std::endl;
        else
            std::cout << "None available" << std::endl;
    }

    std::cout << "Training model with:" << std::endl
        << "- filterbank: " << dataset["filterbank"].asString() << std::endl
        << "- distance: " << dfn << std::endl
        << "- #patches: " << patches.size() << std::endl
        << "- #Cs: " << C.size() << std::endl
        << "- " << (cov ? "covariances" : "correlations") << std::endl;

    if(!cov)
        model.prepare();

    double avg_train = model.train(C, [](){ std::cout << "." << std::flush; });
    std::cout << std::endl << "Average training score *per patch*: " << avg_train << std::endl;

    std::cout << "Saving the model... " << std::flush;
    model.save(argv[2]);
    std::cout << "Done. Cya in predictions!" << std::endl;

    return 0;
}

