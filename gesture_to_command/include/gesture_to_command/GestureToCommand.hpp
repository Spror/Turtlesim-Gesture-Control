#pragma once
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

class GestureToCommand : public rclcpp::Node {
public:
    GestureToCommand();

private:
    void topic_callback(const std_msgs::msg::UInt8 &msg) const;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
};
