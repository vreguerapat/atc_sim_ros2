#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/aeropuerto.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto aeropuerto = std::make_shared<Aeropuerto>();
   
    rclcpp::spin(aeropuerto);
    rclcpp::shutdown();
    return 0;
}