#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class GestureDetection : public rclcpp::Node {
public:
    GestureDetection() : Node("gesture_detecion_node") {
        cap_ = std::make_shared<cv::VideoCapture>(0);

        if (!cap_->isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Camera opening failure");
            rclcpp::shutdown();
        }

        timer_ = create_wall_timer(10ms, std::bind(&GestureDetection::timer_callback, this));
    }

private:
    void timer_callback() {
        cv::Mat frame;
        *cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(get_logger(), "Frame is empty");
        }
        cv::imshow("Camera view", frame);
        cv::waitKey(1);
    }

    std::shared_ptr<cv::VideoCapture> cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GestureDetection>());
    rclcpp::shutdown();

    return 0;
}