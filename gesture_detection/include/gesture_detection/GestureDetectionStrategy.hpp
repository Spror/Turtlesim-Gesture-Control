#pragma once
#include <opencv2/opencv.hpp>

enum class Gesture{
    //TO DO
};

class GestureDetectionStrategy {
public:
    virtual ~GestureDetectionStrategy() = default;

    virtual Gesture gestureDetection(cv::Mat& frame) = 0;
};