#include <GestureToCommand.hpp>

int main()
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GestureToCommand>());
    rclcpp::shutdown();

    return 0;
}