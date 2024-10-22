#pragma once

#include <opencv2/opencv.hpp>

class CameraControl {
public:
    CameraControl() = delete;
    explicit CameraControl(const int index = 0) : camera_(index) {}
    ~CameraControl() = default;

    CameraControl(const CameraControl& other) = delete;
    CameraControl& operator=(const CameraControl& other) = delete;

    CameraControl(CameraControl&& other) = delete;
    CameraControl& operator=(CameraControl&& other) = delete;

    cv::Mat captureFrame();
    bool isOpened() const;

private:
    cv::VideoCapture camera_;
};