#include <GestureToCommand.hpp>

GestureToCommand::GestureToCommand() : Node("gesture_to_command") {
    gesture_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        "gesture_topic", 10,
        std::bind(&GestureToCommand::gestureCallback, this, std::placeholders::_1));

    command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    current_gesture_ = -1;
    gesture_count_ = 0;
}

void GestureToCommand::gestureCallback(const std_msgs::msg::UInt8& msg) {
    auto received_gesture = static_cast<int>(msg.data);

    if (current_gesture_ == received_gesture) {
        gesture_count_++;
    } else {
        gesture_count_ = 0;
        current_gesture_ = received_gesture;
    }

    if (gesture_count_ == 10) {
        sendCommand();
        gesture_count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Received gesture: %d", received_gesture);
    }
}

void GestureToCommand::sendCommand() const {
    auto command = geometry_msgs::msg::Twist();

    switch (current_gesture_) {
    case 0:
        command.linear.x = 0.0;  // stop
        command.angular.z = 0.0;
        break;
    case 1:
        command.linear.x = 1.0;  // move forward
        break;
    case 2:
        command.angular.z = -1.0;  // turn right
        break;
    case 3:
        command.angular.z = 1.0;  // turn left
        break;
    case 4:
        command.linear.x = -1.0;  // move backwards
        break;
    case 5:
        command.linear.x = 0.5;   // circular motion: moderate forward speed
        command.angular.z = 0.5;  // at the same time, moderate rotation speed
        break;
    default:
        RCLCPP_INFO(this->get_logger(), "Unknown gesture");
        break;
    }

    command_publisher_->publish(command);
}
