#include "GestureDetection.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GestureDetection>());
    rclcpp::shutdown();

    return 0;
}