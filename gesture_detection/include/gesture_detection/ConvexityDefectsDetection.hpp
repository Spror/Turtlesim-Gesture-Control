#pragma once
#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "GestureDetectionStrategy.hpp"
#include "HandDetector.hpp"

using pointsVec = std::vector<cv::Point>;
using closestPointsArr = std::array<cv::Point, 2>;
class ConvexityDefectsDetection : public GestureDetectionStrategy {
public:
    ConvexityDefectsDetection() = delete;
    explicit ConvexityDefectsDetection(const rclcpp::Logger& logger,
                                       cv::Scalar lower_bound,
                                       cv::Scalar upper_bound)
        : logger_{logger},
          handDetecotr_ptr_(std::make_unique<HandDetector>(lower_bound, upper_bound)) {};

    Gesture gestureDetection(const cv::Mat& frame, cv::Mat& output) override;

private:
    const rclcpp::Logger logger_;
    std::unique_ptr<HandDetector> handDetecotr_ptr_;

    inline double pointDistanceOnX(const cv::Point& a, const cv::Point& b) const;
    inline double pointDistance(const cv::Point& a, const cv::Point& b) const;

    Gesture toGesture(const int fingersNum) const;

    cv::Point calculateMedian(const pointsVec& group) const;
    closestPointsArr findClosestOnX(pointsVec points, const cv::Point pivot) const;
    void reducePointsToMedians(pointsVec& points, const double maxDistance) const;
    void compressPointsByDistance(pointsVec& points, const double maxDistance) const;
    bool isFinger(const cv::Point& tipPoint,
                  const cv::Point& defPoint_1,
                  const cv::Point& defPoint_2,
                  const cv::Point& palmCenter,
                  const double minDistance) const;
};