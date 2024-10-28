#include "HandDetector.hpp"

cv::Mat HandDetector::detectHand(const cv::Mat& frame) {
    auto hand = imageProcess(frame);

    return hand;
}

cv::Mat HandDetector::imageProcess(const cv::Mat& frame) {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat skinMask;
    cv::inRange(hsv, lowerBound, upperBound, skinMask);

    cv::erode(skinMask, skinMask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(skinMask, skinMask, cv::Mat(), cv::Point(-1, -1), 2);

    return skinMask;
}