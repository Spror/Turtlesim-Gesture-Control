#include "CameraControl.hpp"

bool CameraControl::isOpened() const {
    return camera_.isOpened();
}

cv::Mat CameraControl::captureFrame() {
    cv::Mat frame;

    if (camera_.isOpened()) {
        camera_ >> frame;
    }

    return frame;
}
