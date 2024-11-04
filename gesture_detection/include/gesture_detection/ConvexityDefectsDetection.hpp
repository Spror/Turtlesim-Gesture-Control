#pragma once
#include <memory>
#include <vector>
#include "GestureDetectionStrategy.hpp"
#include "HandDetector.hpp"

class ConvexityDefectsDetection : public GestureDetectionStrategy {
public:
    ConvexityDefectsDetection() = delete;
    explicit ConvexityDefectsDetection(cv::Scalar lower_bound, cv::Scalar upper_bound)
        : handDetecotr_ptr_(std::make_unique<HandDetector>(lower_bound, upper_bound)) {};

    Gesture gestureDetection(cv::Mat& frame) override;

private:
    std::unique_ptr<HandDetector> handDetecotr_ptr_;

};