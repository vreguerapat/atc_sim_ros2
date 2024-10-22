#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/visualizador.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto visualizador = std::make_shared<Visualizador>();
    rclcpp::spin(visualizador);
    rclcpp::shutdown();
    return 0;
}