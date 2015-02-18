#pragma once

#include <iomanip>
#include <string>
#include <sstream>

#include <opencv2/opencv.hpp>

namespace warco {

template<typename T>
std::string to_s(const T& t)
{
    std::ostringstream ss;
    ss << t;
    return ss.str();
}

// Based upon http://stackoverflow.com/a/10168511
template<>
inline std::string to_s<cv::Mat>(const cv::Mat& m)
{
    std::ostringstream ss("Matrix: ");

    int type = m.type();

    // Channel type
    switch(type & CV_MAT_DEPTH_MASK) {
        case CV_8U:  ss << "8U"; break;
        case CV_8S:  ss << "8S"; break;
        case CV_16U: ss << "16U"; break;
        case CV_16S: ss << "16S"; break;
        case CV_32S: ss << "32S"; break;
        case CV_32F: ss << "32F"; break;
        case CV_64F: ss << "64F"; break;
        default:     ss << "User"; break;
    }

    // Channel count
    ss << "C" << 1 + (type >> CV_CN_SHIFT);

    // Size
    ss << " " << m.cols << "x" << m.rows;

    // Min/Max
    double min, max;
    minMaxIdx(m, &min, &max);
    ss << " values in [" << min << ".." << max << "]";

    return ss.str();
}

template<typename T>
inline std::string to_s(const std::vector<T>& v)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << "[";

    auto Nm1 = v.size() - 1;
    for(unsigned i = 0 ; i < Nm1 ; ++i) {
        ss << v[i] << ", ";
    }
    ss << v[Nm1] << "]";

    return ss.str();
}

} // namespace warco

