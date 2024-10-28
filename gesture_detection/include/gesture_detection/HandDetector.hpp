#pragma once
#include <opencv2/opencv.hpp>

class HandDetector {
public:
    explicit HandDetector(cv::Scalar lower_bound, cv::Scalar upper_bound)
        : lower_bound_(lower_bound), upper_bound_(upper_bound) {};

    cv::Mat detectHand(const cv::Mat& frame);

private:
    cv::Scalar lower_bound_;
    cv::Scalar upper_bound_;
};