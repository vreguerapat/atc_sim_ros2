#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/flight.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include <vector>
#include <cmath>

class ATC : public rclcpp::Node {
    public:
        ATC() : Node("atc") {
            aviones_sub_ = this->create_subscription<atc_sim_ros2::msg::ListaAviones>( "lista_aviones", 10, std::bind(&ATC::checkCollisions, this, std::placeholders::_1));
        }
    
    private:
        rclcpp::Subscription<atc_sim_ros2::msg::ListaAviones>::SharedPtr aviones_sub_;

        void checkCollisions(const atc_sim_ros2::msg::ListaAviones::SharedPtr msg) {
            const auto& aviones = msg->aviones;

            for (size_t i = 0; i < aviones.size(); i++) {
                for (size_t j = i+1; j < aviones.size(); j++) {
                    double distance = calculateDistance(aviones[i], aviones[j]);
                    if (distance < 5.0) { // Umbral de distancia para colision
                        RCLCPP_WARN(this->get_logger(), "Posible colision entre Avion %s y Avion %s", aviones[i].id.c_str(), aviones[j].id.c_str());
                    }
                }
            }
        }
        
        double calculateDistance(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2) {
            double dx = avion1.posx - avion2.posx;
            double dy = avion1.posy - avion2.posy;
            double dz = avion1.posz - avion2.posz;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            return distance;
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ATC>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}