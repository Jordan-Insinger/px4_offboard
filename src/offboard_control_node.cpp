#include "offboardControl.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<offboardControl>());
    rclcpp::shutdown();
    return 0;
}
