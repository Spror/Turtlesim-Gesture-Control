#include "ConvexityDefectsDetection.hpp"
#include <limits.h>
#include <algorithm>

#define FINGER_SIZE_SCALING 0.3
#define NEIGHBOR_DISTANCE_SCALING 0.1

 double ConvexityDefectsDetection::pointDistanceOnX(const cv::Point a,
                                                          const cv::Point b) const {
    return std::abs(a.x - b.x);
}

 double ConvexityDefectsDetection::pointDistance(const cv::Point a, const cv::Point b) const {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
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
    if (points.size() < 2) {
        return {};
    }

    std::partial_sort(points.begin(), points.begin() + 2, points.end(),
                      [pivot, this](const auto& a, const auto& b) {
                          auto distance_x_a = pointDistanceOnX(pivot, a);
                          auto distance_x_b = pointDistanceOnX(pivot, b);
                          if (distance_x_a != distance_x_b) {
                              return distance_x_a < distance_x_b;
                          } else {
                              return pointDistance(pivot, a) < pointDistance(pivot, b);
                          }
                      });

    return {points[0], points[1]};
}

Gesture ConvexityDefectsDetection::gestureDetection(const cv::Mat& frame, cv::Mat& output) {
    if (frame.empty()) {
        RCLCPP_ERROR(logger_, "Input frame is empty.");
        return Gesture::No_gesture;
    }

    output = handDetecotr_ptr_->detectHand(frame);

    if (output.empty()) {
        RCLCPP_ERROR(logger_, "Hand extraction failure.");
        return Gesture::No_gesture;
    }

    pointsVec hull;
    std::vector<int> hull_ints;

    auto contour = handDetecotr_ptr_->getHandContour();

    cv::convexHull(contour, hull, true);
    cv::convexHull(contour, hull_ints, false);

    std::vector<cv::Vec4i> defects;

    // At least 3 points to find defects
    if (hull_ints.size() < 3) {
        RCLCPP_ERROR(logger_, "Defects finding failure.");
        return Gesture::No_gesture;
    }

    cv::convexityDefects(contour, hull_ints, defects);
    cv::cvtColor(output, output, cv::COLOR_GRAY2BGR);

    cv::Rect bounding_rectangle = cv::boundingRect(hull);
    cv::rectangle(output, bounding_rectangle.tl(), bounding_rectangle.br(), cv::Scalar(255), 2, 8,
                  0);

    cv::Point center_bounding_rect((bounding_rectangle.tl().x + bounding_rectangle.br().x) / 2,
                                   (bounding_rectangle.tl().y + bounding_rectangle.br().y) / 2);

    pointsVec start_points;
    pointsVec far_points;

    for (const auto& defect : defects) {
        start_points.push_back(contour[defect.val[0]]);

        if (pointDistance(contour[defect.val[2]], center_bounding_rect) <
            bounding_rectangle.height * FINGER_SIZE_SCALING) {
            far_points.push_back(contour[defect.val[2]]);
        }
    }

    start_points = compressPointsByNeighborhood(
        start_points, bounding_rectangle.width * NEIGHBOR_DISTANCE_SCALING);

    far_points = compressPointsByNeighborhood(far_points,
                                              bounding_rectangle.width * NEIGHBOR_DISTANCE_SCALING);

    auto kk = findClosestOnX(far_points, start_points[1]);
    for (const auto& point : start_points) {
        cv::circle(output, point, 2, cv::Scalar(0, 0, 255), -1);
    }

    for (const auto& point : kk) {
        cv::circle(output, point, 2, cv::Scalar(0, 255, 0), -1);
    }

    return Gesture::No_gesture;
}

pointsVec ConvexityDefectsDetection::compressPointsByNeighborhood(const pointsVec& points,
                                                                  const double maxDistance) const {
    pointsVec median_points;

    if (points.empty() || maxDistance <= 0)
        return median_points;

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

    return median_points;
}
