#include "HandDetector.hpp"
#include <vector>

cv::Mat HandDetector::detectHand(const cv::Mat& frame) {
    auto skinMask = imageProcess(frame);
    auto hand = extractHand(skinMask);

    return hand;
}

cv::Mat HandDetector::imageProcess(const cv::Mat& frame) {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat skinMask;
    cv::inRange(hsv, lower_bound_, upper_bound_, skinMask);

    cv::erode(skinMask, skinMask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(skinMask, skinMask, cv::Mat(), cv::Point(-1, -1), 2);

    return skinMask;
}

cv::Mat HandDetector::extractHand(const cv::Mat& frame) const {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int largest_contour_index = -1;
    double max_area = 0;

    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            largest_contour_index = static_cast<int>(i);
        }
    }

    cv::Mat handMask = cv::Mat::zeros(frame.size(), CV_8UC1);
    if (largest_contour_index != -1) {
        cv::drawContours(handMask, contours, largest_contour_index, cv::Scalar(255), cv::FILLED);
    }

    cv::Mat hand;
    frame.copyTo(hand, handMask);

    return hand;
}
