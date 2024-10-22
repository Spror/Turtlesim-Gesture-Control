#include "CameraControl.hpp"

CameraControl::CameraControl(int index) : index_(index) {
    camera_.open(index_);
}

bool CameraControl::isOpened() {
    return camera_.isOpened();
}

cv::Mat CameraControl::captureFrame() {
    cv::Mat frame;

    if (camera_.isOpened()) {
        camera_ >> frame;
    }

    return frame;
}
