#pragma once
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"


class GestureToCommand : public rclcpp::Node {
public:
    GestureToCommand();

private:
    void gestureCallback(const std_msgs::msg::UInt8 &msg);
    void sendCommand() const;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gesture_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;

    int current_gesture_;
    int gesture_count_;
};
