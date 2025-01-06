#include <GestureDetection.hpp>
#include <algorithm>

GestureDetection::GestureDetection() : Node("gesture_detection"), finger_count_(0) {
    this->declare_parameter<std::vector<long>>("lower_bound", {0, 0, 0});
    this->declare_parameter<std::vector<long>>("upper_bound", {255, 255, 255});

    std::vector<long> lower_bound_vec, upper_bound_vec;
    this->get_parameter("lower_bound", lower_bound_vec);
    this->get_parameter("upper_bound", upper_bound_vec);

    lower_bound_ =
        cv::Scalar(static_cast<double>(lower_bound_vec[0]), static_cast<double>(lower_bound_vec[1]),
                   static_cast<double>(lower_bound_vec[2]));
    upper_bound_ =
        cv::Scalar(static_cast<double>(upper_bound_vec[0]), static_cast<double>(upper_bound_vec[1]),
                   static_cast<double>(upper_bound_vec[2]));

    publisher_ = this->create_publisher<std_msgs::msg::UInt8>("gesture_topic", 10);
    hand_detector_p_ = std::make_shared<HandDetector>(lower_bound_, upper_bound_);

    camera_p_ = std::make_shared<CameraControl>(0);
    strategy_detection_p_ =
        std::make_shared<ConvexityDefectsDetection>(this->get_logger(), hand_detector_p_);

    param_callback_ = this->add_on_set_parameters_callback(
        std::bind(&GestureDetection::parametersCallback, this, std::placeholders::_1));

    if (!camera_p_->isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Camera opening failure");
        rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Initial lower_bound: [%f, %f, %f]", lower_bound_[0],
                lower_bound_[1], lower_bound_[2]);
    RCLCPP_INFO(this->get_logger(), "Initial upper_bound: [%f, %f, %f]", upper_bound_[0],
                upper_bound_[1], upper_bound_[2]);

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

rcl_interfaces::msg::SetParametersResult GestureDetection::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters updated successfuly.";

    auto is_in_range = [](const auto& x) {
        if (x >= 0 && x <= 255) {
            return true;
        } else {
            return false;
        }
    };

    for (const auto& param : parameters) {
        if (param.get_name() == "lower_bound" &&
            param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
            auto new_values_array = param.as_integer_array();
            if (new_values_array.size() == 3) {
                if (std::all_of(begin(new_values_array), end(new_values_array), is_in_range)) {
                    lower_bound_ = cv::Scalar(static_cast<uint8_t>(new_values_array[0]),
                                              static_cast<uint8_t>(new_values_array[1]),
                                              static_cast<uint8_t>(new_values_array[3]));
                    hand_detector_p_->setLowerBound(lower_bound_);
                    RCLCPP_INFO(this->get_logger(), "Updated lower_bound [%f, %f, %f]",
                                lower_bound_[0], lower_bound_[1], lower_bound_[2]);
                } else {
                    result.successful = false;
                    result.reason = "Invalid numbers for parameters.";
                }
            } else {
                result.successful = false;
                result.reason = "Invalid size for parameters.";
            }
        } else if (param.get_name() == "upper_bound" &&
                   param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
            auto new_values_array = param.as_integer_array();
            if (new_values_array.size() == 3) {
                if (std::all_of(begin(new_values_array), end(new_values_array), is_in_range)) {
                    upper_bound_ = cv::Scalar(static_cast<uint8_t>(new_values_array[0]),
                                              static_cast<uint8_t>(new_values_array[1]),
                                              static_cast<uint8_t>(new_values_array[3]));
                    hand_detector_p_->setUpperBound(upper_bound_);
                    RCLCPP_INFO(this->get_logger(), "Updated lower_bound [%f, %f, %f]",
                                upper_bound_[0], upper_bound_[1], upper_bound_[2]);
                } else {
                    result.successful = false;
                    result.reason = "Invalid numbers for parameters.";
                }
            } else {
                result.successful = false;
                result.reason = "Invalid size for parameters.";
            }
        } else {
            result.successful = false;
            result.reason = "Invalid parameters.";
        }
    }

    return result;
}