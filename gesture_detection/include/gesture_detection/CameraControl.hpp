#pragma once

#include <opencv2/opencv.hpp>

class CameraControl {
public:
    CameraControl() = delete;
    CameraControl(int index = 0);
    cv::Mat captureFrame();
    bool isOpened();

private:
    cv::VideoCapture camera_;
    const int index_;
};