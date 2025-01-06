#include "ConvexityDefectsDetection.hpp"
#include <limits.h>
#include <algorithm>
#include <iostream>
#include <map>

#define FINGER_SIZE_SCALING 0.3
#define NEIGHBOR_DISTANCE_SCALING_FAR 0.1
#define NEIGHBOR_DISTANCE_SCALING_START 0.18

inline double ConvexityDefectsDetection::pointDistanceOnX(const cv::Point& a,
                                                          const cv::Point& b) const {
    return std::abs(a.x - b.x);
}

inline double ConvexityDefectsDetection::pointDistance(const cv::Point& a,
                                                       const cv::Point& b) const {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

void ConvexityDefectsDetection::drawDebugInfo(cv::Mat& frame,
                                              const pointsVec& startPoints,
                                              const pointsVec& farPoints,
                                              const cv::Rect& boundingRect,
                                              const cv::Point& palmCenter) const {
    cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    cv::rectangle(frame, boundingRect.tl(), boundingRect.br(), cv::Scalar(255), 2, 8, 0);

    for (const auto& point : farPoints) {
        cv::circle(frame, point, 4, cv::Scalar(0, 255, 0), -1);
    }
    for (const auto& point : startPoints) {
        cv::circle(frame, point, 4, cv::Scalar(0, 0, 255), -1);
    }

    cv::circle(frame, palmCenter, 10, cv::Scalar(255, 0, 0), -1);

    auto text = "Fingers num: " + std::to_string(fingers_num_);
    cv::Point position(50, 50);
    cv::putText(frame, text, position, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
}

std::pair<pointsVec, std::vector<int>> ConvexityDefectsDetection::calculateConvexHull(
    const pointsVec& contour) const {
    pointsVec hull;
    std::vector<int> hullInts;

    cv::convexHull(contour, hull, true);
    cv::convexHull(contour, hullInts, false);

    return {hull, hullInts};
}

std::vector<cv::Vec4i> ConvexityDefectsDetection::calculateConvexityDefects(
    const pointsVec& contour,
    const std::vector<int>& hullInts) const {
    std::vector<cv::Vec4i> defects;
    cv::convexityDefects(contour, hullInts, defects);

    return defects;
}

std::pair<pointsVec, pointsVec> ConvexityDefectsDetection::processDefects(
    const std::vector<cv::Vec4i>& defects,
    const pointsVec& contour,
    const cv::Rect& boundingRect,
    const cv::Point& center) const {
    pointsVec startPoints;
    pointsVec farPoints;

    for (const auto& defect : defects) {
        if (contour[defect.val[0]].y < center.y) {
            startPoints.push_back(contour[defect.val[0]]);
        }

        double distance = pointDistance(contour[defect.val[2]], center);
        if (distance < boundingRect.height * FINGER_SIZE_SCALING) {
            if (contour[defect.val[2]].y < center.y) {
                farPoints.push_back(contour[defect.val[2]]);
            }
        }
    }

    reducePointsToMedians(startPoints, boundingRect.width * NEIGHBOR_DISTANCE_SCALING_START);
    reducePointsToMedians(farPoints, boundingRect.width * NEIGHBOR_DISTANCE_SCALING_FAR);
    compressPointsByDistance(startPoints, boundingRect.height * NEIGHBOR_DISTANCE_SCALING_FAR);

    return {startPoints, farPoints};
}

Gesture ConvexityDefectsDetection::countFingers(const pointsVec& startPoints,
                                                const pointsVec& farPoints,
                                                const cv::Rect& boundingRect,
                                                const cv::Point& palmCenter) {
    if (farPoints.size() < 1) {
        fingers_num_ = 0;
        return Gesture::No_gesture;
    }

    pointsVec fingerPoints;

    for (const auto& point : startPoints) {
        closestPointsArr closestPoints = findClosestOnX(farPoints, point);
        if (isFinger(point, closestPoints[0], closestPoints[1], palmCenter,
                     boundingRect.height * FINGER_SIZE_SCALING)) {
            fingerPoints.push_back(point);
        }
    }

    return toGesture(static_cast<int>(fingerPoints.size()));
}

Gesture ConvexityDefectsDetection::toGesture(const int fingersNum) {
    switch (fingersNum) {
    case 1:
        fingers_num_ = 1;
        return Gesture::One;
        break;
    case 2:
        fingers_num_ = 2;
        return Gesture::Two;
        break;
    case 3:
        fingers_num_ = 3;
        return Gesture::Three;
        break;
    case 4:
        fingers_num_ = 4;
        return Gesture::Four;
        break;
    case 5:
        fingers_num_ = 5;
        return Gesture::Five;
        break;

    default:
        fingers_num_ = 0;
        return Gesture::No_gesture;
        break;
    }
}

cv::Point ConvexityDefectsDetection::calculateMedian(const pointsVec& group) const {
    cv::Point sum(0, 0);
    for (const auto& point : group) {
        sum += point;
    }

    return sum / static_cast<int>(group.size());
}

closestPointsArr ConvexityDefectsDetection::findClosestOnX(pointsVec points,
                                                           const cv::Point pivot) const {
    if (points.size() < 3) {
        return {};
    }

    std::partial_sort(begin(points), begin(points) + 1, end(points),
                      [pivot, this](const auto& a, const auto& b) {
                          return pointDistanceOnX(pivot, a) <= pointDistanceOnX(pivot, b);
                      });

    return {points[0], points[1]};
}

Gesture ConvexityDefectsDetection::gestureDetection(const cv::Mat& frame, cv::Mat& output) {
    auto gesture_to_return = Gesture::No_gesture;
    if (frame.empty()) {
        RCLCPP_ERROR(logger_, "Input frame is empty.");
        return gesture_to_return;
    }

    output = handDetecotr_ptr_->detectHand(frame);

    if (output.empty()) {
        RCLCPP_ERROR(logger_, "Hand extraction failure.");
        return gesture_to_return;
    }

    auto contour = handDetecotr_ptr_->getHandContour();
    auto [hull, hull_ints] = calculateConvexHull(contour);

    // At least 3 points to find defects
    if (hull_ints.size() < 3) {
        RCLCPP_ERROR(logger_, "Defects finding failure.");
        return gesture_to_return;
    }

    auto defects = calculateConvexityDefects(contour, hull_ints);

    cv::Rect bounding_rectangle = cv::boundingRect(hull);
    cv::Point center((bounding_rectangle.tl().x + bounding_rectangle.br().x) / 2,
                     (bounding_rectangle.tl().y + bounding_rectangle.br().y) / 2);

    cv::rectangle(output, bounding_rectangle.tl(), bounding_rectangle.br(), cv::Scalar(255), 2, 8,
                  0);

    auto [start_points, far_points] = processDefects(defects, contour, bounding_rectangle, center);
    gesture_to_return = countFingers(start_points, far_points, bounding_rectangle, center);
    drawDebugInfo(output, start_points, far_points, bounding_rectangle, center);

    return gesture_to_return;
}

void ConvexityDefectsDetection::reducePointsToMedians(pointsVec& points,
                                                      const double maxDistance) const {
    pointsVec median_points;

    if (points.empty() || maxDistance <= 0)
        return;

    pointsVec current_group;
    current_group.push_back(points[0]);

    for (size_t i = 1; i < points.size(); ++i) {
        if (pointDistance(current_group.back(), points[i]) > maxDistance) {
            median_points.push_back(calculateMedian(current_group));

            current_group.clear();
        }
        current_group.push_back(points[i]);
    }

    if (!current_group.empty()) {
        median_points.push_back(calculateMedian(current_group));
    }

    points = std::move(median_points);
}

void ConvexityDefectsDetection::compressPointsByDistance(pointsVec& points,
                                                         const double maxDistance) const {
    auto is_close = [maxDistance, this](const auto& point_a, const auto& point_b) {
        return pointDistance(point_a, point_b) < maxDistance;
    };

    pointsVec compressedPoints;
    compressedPoints.reserve(points.size());

    for (const auto& point : points) {
        if (std::none_of(begin(compressedPoints), end(compressedPoints),
                         [&point, &is_close](const auto& existing_point) {
                             return is_close(point, existing_point);
                         })) {
            compressedPoints.push_back(point);
        }
    }

    points = std::move(compressedPoints);
}

bool ConvexityDefectsDetection::isFinger(const cv::Point& tipPoint,
                                         const cv::Point& defPoint_1,
                                         const cv::Point& defPoint_2,
                                         const cv::Point& palmCenter,
                                         const double minDistance) const {
    // Fingertip should not be under the two defects points
    auto diff_y_1 = tipPoint.y - defPoint_1.y;
    auto diff_y_2 = tipPoint.y - defPoint_2.y;
    if (diff_y_1 > 0 && diff_y_2 > 0) {
        return false;
    }

    // // Distance from palm to fingertip should be greater than minPalmTipDistance
    auto palmTip_distance = pointDistance(tipPoint, palmCenter);
    if (palmTip_distance < minDistance) {
        return false;
    }

    return true;
}
