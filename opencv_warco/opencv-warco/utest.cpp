#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <time.h>

#include <opencv2/opencv.hpp>

#include "covcorr.hpp"
#include "cvutils.hpp"
#include "dists.hpp"
#include "model.hpp"

int main(int argc, char** argv)
{
    auto seed = argc == 2 ? strtoul(argv[1], nullptr, 0) : time(nullptr);
    std::cout << "Seed is " << seed << std::endl;
    cv::theRNG().state = seed;
    srand(seed);

    warco::test_cv_utils();
    warco::test_covcorr();
    warco::test_dists();
    warco::test_model();

    return 0;
}
