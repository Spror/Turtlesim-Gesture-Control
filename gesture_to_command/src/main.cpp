#include <GestureToCommand.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GestureToCommand>());
    rclcpp::shutdown();

    return 0;
}