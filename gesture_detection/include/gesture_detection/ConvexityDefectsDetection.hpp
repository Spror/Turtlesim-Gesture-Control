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
                                       std::shared_ptr<HandDetector> handDetecotr_ptr)
        : logger_{logger},
          handDetecotr_ptr_(handDetecotr_ptr) {};

    Gesture gestureDetection(const cv::Mat& frame, cv::Mat& output) override;

private:
    const rclcpp::Logger logger_;
    std::shared_ptr<HandDetector> handDetecotr_ptr_;
    int fingers_num_ = 0;

    inline double pointDistanceOnX(const cv::Point& a, const cv::Point& b) const;
    inline double pointDistance(const cv::Point& a, const cv::Point& b) const;

    void drawDebugInfo(cv::Mat& frame,
                       const pointsVec& startPoints,
                       const pointsVec& farPoints,
                       const cv::Rect& boundingRect,
                       const cv::Point& palmCenter) const;

    std::pair<pointsVec, std::vector<int>> calculateConvexHull(const pointsVec& contour) const;
    std::vector<cv::Vec4i> calculateConvexityDefects(const pointsVec& contour,
                                                     const std::vector<int>& hullInts) const;

    std::pair<pointsVec, pointsVec> processDefects(const std::vector<cv::Vec4i>& defects,
                                                   const pointsVec& contour,
                                                   const cv::Rect& boundingRect,
                                                   const cv::Point& center) const;

    Gesture toGesture(const int fingersNum);
    Gesture countFingers(const pointsVec& startPoints,
                         const pointsVec& farPoints,
                         const cv::Rect& boundingRect,
                         const cv::Point& palmCenter);

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