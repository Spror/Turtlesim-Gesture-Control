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
    GestureDetection() : Node("gesture_detecion_node"), finger_count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::UInt8>("gesture_topic", 10);

        camera_p_ = std::make_shared<CameraControl>(0);
        strategy_detection_p_ = std::make_shared<ConvexityDefectsDetection>(
            this->get_logger(), cv::Scalar(100, 119, 120), cv::Scalar(216, 255, 255));

        if (!camera_p_->isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Camera opening failure");
            rclcpp::shutdown();
        }

        timer_ = create_wall_timer(20ms, std::bind(&GestureDetection::publishGesture, this));
    }

private:
    void publishGesture() {
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
   
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;

    std::shared_ptr<CameraControl> camera_p_;
    std::shared_ptr<GestureDetectionStrategy> strategy_detection_p_;
        
    uint8_t finger_count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GestureDetection>());
    rclcpp::shutdown();

    return 0;
}