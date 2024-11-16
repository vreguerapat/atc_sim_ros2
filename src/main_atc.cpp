#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/atc.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ATC>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}