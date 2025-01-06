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
    uint8_t finger_count_;
    void publishGesture();

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

    cv::Scalar lower_bound_;
    cv::Scalar upper_bound_;

    std::shared_ptr<CameraControl> camera_p_;
    std::shared_ptr<HandDetector> hand_detector_p_;
    std::shared_ptr<GestureDetectionStrategy> strategy_detection_p_;
};