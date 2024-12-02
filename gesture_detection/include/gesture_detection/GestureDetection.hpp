#pragma once

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include "CameraControl.hpp"
#include "ConvexityDefectsDetection.hpp"

#include "std_msgs/msg/u_int8.hpp"

using namespace std::chrono_literals;

class GestureDetection : public rclcpp::Node {
public:
    GestureDetection();

private:
    void publishGesture();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;

    std::shared_ptr<CameraControl> camera_p_;
    std::shared_ptr<GestureDetectionStrategy> strategy_detection_p_;

    uint8_t finger_count_;
};