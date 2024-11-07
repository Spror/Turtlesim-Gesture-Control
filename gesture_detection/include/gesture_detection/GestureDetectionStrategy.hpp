#pragma once
#include <opencv2/opencv.hpp>

enum class Gesture {
    No_gesture,
    One,
    Two,
    Three,
    Four,
    Five
};

class GestureDetectionStrategy {
public:
    virtual ~GestureDetectionStrategy() = default;

    virtual Gesture gestureDetection(const cv::Mat& frame, cv::Mat& output) = 0;
};