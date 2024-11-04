#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class HandDetector {
public:
    HandDetector() = delete;
    explicit HandDetector(cv::Scalar lower_bound, cv::Scalar upper_bound)
        : lower_bound_(lower_bound), upper_bound_(upper_bound) {};

    cv::Mat detectHand(const cv::Mat& frame);
    std::vector<cv::Point> getHandContour() const { return handContour_; };

private:
    cv::Scalar lower_bound_;
    cv::Scalar upper_bound_;

    std::vector<cv::Point> handContour_;

    cv::Mat imageProcess(const cv::Mat& frame);
    cv::Mat extractHand(const cv::Mat& frame);
};