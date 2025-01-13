#include <GestureToCommand.hpp>

GestureToCommand::GestureToCommand() : Node("gesture_to_command") {
    subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
        "gesture_topic", 10, std::bind(&GestureToCommand::topic_callback, this, std::placeholders::_1));
}

void GestureToCommand::topic_callback(const std_msgs::msg::UInt8 &msg) const {
    RCLCPP_INFO(this->get_logger(), "Received gesture: %d", msg.data);
}
