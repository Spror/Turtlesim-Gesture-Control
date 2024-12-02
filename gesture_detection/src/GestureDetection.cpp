#include <GestureDetection.hpp>

GestureDetection::GestureDetection() : Node("gesture_detecion_node"), finger_count_(0) {
    this->declare_parameter<std::vector<uint8_t>>("lower_bound", {0, 0, 0});
    this->declare_parameter<std::vector<uint8_t>>("upper_bound", {255, 255, 255});

    std::vector<uint8_t> lower_bound_vec, upper_bound_vec;
    this->get_parameter("lower_bound", lower_bound_vec);
    this->get_parameter("upper_bound", upper_bound_vec);

    cv::Scalar lower_bound(static_cast<double>(lower_bound_vec[0]),
                           static_cast<double>(lower_bound_vec[1]),
                           static_cast<double>(lower_bound_vec[2]));
    cv::Scalar upper_bound(static_cast<double>(upper_bound_vec[0]),
                           static_cast<double>(upper_bound_vec[1]),
                           static_cast<double>(upper_bound_vec[2]));

    publisher_ = this->create_publisher<std_msgs::msg::UInt8>("gesture_topic", 10);

    camera_p_ = std::make_shared<CameraControl>(0);
    strategy_detection_p_ =
        std::make_shared<ConvexityDefectsDetection>(this->get_logger(), lower_bound, upper_bound);

    if (!camera_p_->isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Camera opening failure");
        rclcpp::shutdown();
    }

    timer_ = create_wall_timer(20ms, std::bind(&GestureDetection::publishGesture, this));
}

void GestureDetection::publishGesture() {
    auto message = std_msgs::msg::UInt8();
    auto frame = camera_p_->captureFrame();
    cv::Mat hand;

    if (frame.empty()) {
        RCLCPP_WARN(get_logger(), "Frame is empty");
    }

    auto current_gesture = strategy_detection_p_->gestureDetection(frame, hand);

    finger_count_ = static_cast<uint8_t>(current_gesture);
    message.data = finger_count_;
    RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.data);
    publisher_->publish(message);

    cv::imshow("Camera view", frame);
    cv::imshow("Processed view", hand);
    cv::waitKey(1);
}