#pragma once
#include <opencv2/opencv.hpp>

enum class Gesture { 
    No_gesture = 0,
    One = 1,
    Two = 2,
    Three = 3,
    Four = 4,
    Five = 5 
};

class GestureDetectionStrategy {
public:
    virtual ~GestureDetectionStrategy() = default;

    virtual Gesture gestureDetection(const cv::Mat& frame, cv::Mat& output) = 0;
};