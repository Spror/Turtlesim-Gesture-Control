#include <GestureToCommand.hpp>

GestureToCommand::GestureToCommand() : Node("gesture_to_command") {
    gesture_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        "gesture_topic", 10, std::bind(&GestureToCommand::gestureCallback, this, std::placeholders::_1));
}

void GestureToCommand::gestureCallback(const std_msgs::msg::UInt8 &msg) const {
    RCLCPP_INFO(this->get_logger(), "Received gesture: %d", msg.data);
}
