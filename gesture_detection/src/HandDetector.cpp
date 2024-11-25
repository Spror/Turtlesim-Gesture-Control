#include "HandDetector.hpp"

cv::Mat HandDetector::detectHand(const cv::Mat& frame) {
    auto output = imageProcess(frame);
    return extractHand(output);
}

cv::Mat HandDetector::imageProcess(const cv::Mat& frame) const {
    cv::Mat YCrCb;
    cv::cvtColor(frame, YCrCb, cv::COLOR_BGR2YCrCb);

    cv::Mat skinMask;
    cv::inRange(YCrCb, lower_bound_, upper_bound_, skinMask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    cv::erode(skinMask, skinMask, kernel, cv::Point(-1, -1), 1);
    cv::dilate(skinMask, skinMask, kernel, cv::Point(-1, -1), 2);

    return skinMask;
}

cv::Mat HandDetector::extractHand(const cv::Mat& frame) {
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
        handContour_ = contours[largest_contour_index];
        cv::drawContours(handMask, contours, largest_contour_index, cv::Scalar(255), cv::FILLED);
    }

    cv::Mat hand;
    frame.copyTo(hand, handMask);

    return hand;
}
