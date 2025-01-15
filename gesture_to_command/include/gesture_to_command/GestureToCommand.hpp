#pragma once
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"

class GestureToCommand : public rclcpp::Node {
public:
    GestureToCommand();

private:
    void gestureCallback(const std_msgs::msg::UInt8 &msg) const;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gesture_subscriber_;

};
