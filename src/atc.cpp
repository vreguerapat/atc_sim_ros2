#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/flight.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include "atc_sim_ros2/avion.hpp"
#include "atc_sim_ros2/msg/waypoint.hpp"
#include "atc_sim_ros2/msg/waypoint_update.hpp"
#include <vector>
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ATC : public rclcpp::Node {
    public:
        ATC() : Node("atc") {
            aviones_sub_ = this->create_subscription<atc_sim_ros2::msg::ListaAviones>( "lista_aviones", 10, std::bind(&ATC::manageRoutes, this, std::placeholders::_1));

            waypoints_pub_ = this->create_publisher<atc_sim_ros2::msg::WaypointUpdate>("waypoint_update", 10);
            // Se definen los destinos, los mismos que los waypoints de visualizador.cpp
            atc_sim_ros2::msg::Waypoint destino1, destino2;
            destino1.x = 5.0; destino1.y = 5.0; destino1.z = 5.0;
            destino2.x = 0.0; destino2.y = 0.0; destino2.z = 5.0;

            waypoints_destino_ = {destino1, destino2};
            contador_destinos_ =  {0,0};                                                                                                                                                                                                                                                   
        }
    
    private:
        rclcpp::Subscription<atc_sim_ros2::msg::ListaAviones>::SharedPtr aviones_sub_;
        rclcpp::Publisher<atc_sim_ros2::msg::WaypointUpdate>::SharedPtr waypoints_pub_;
        std::vector<atc_sim_ros2::msg::Waypoint> waypoints_destino_;
        std::vector<int> contador_destinos_;

        void manageRoutes(const atc_sim_ros2::msg::ListaAviones::SharedPtr lista_aviones) {
            for (auto& avion_msg : lista_aviones->aviones) {             
                // Se verifica si el avion ya tiene waypoints asignados
                if (avion_msg.waypoints.empty()) {
                    // Se le asigna destino
                    assignRoute(avion_msg);
                }
            }

        }

        void assignRoute (atc_sim_ros2::msg::Flight& avion_msg) {
            RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Avion %s tiene %zu waypoints antes de asignar ruta", avion_msg.id.c_str(), avion_msg.waypoints.size());
            atc_sim_ros2::msg::Waypoint start;
            start.x = avion_msg.posx;
            start.y = avion_msg.posy;
            start.z = avion_msg.posz;
            // Se selecciona el destino menos asignado
            int destino_index = selectLeastAssignedDestination();

            atc_sim_ros2::msg::Waypoint destino = waypoints_destino_[destino_index];
            auto waypoints = generateIntermediateWaypoints(start, destino, 2);

            waypoints.push_back(destino);

            if (!waypoints.empty()) {
                RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Publicando waypoints para el avion %s", avion_msg.id.c_str());
                atc_sim_ros2::msg::WaypointUpdate update_msg;
                update_msg.avion_id = avion_msg.id;
                update_msg.waypoints = waypoints;

                waypoints_pub_->publish(update_msg);
                RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Waypoints publicados para el avion %s", avion_msg.id.c_str());
                contador_destinos_[destino_index]++; // Se actualiza contador
            }
        }

        int selectLeastAssignedDestination() {
            int min_index = 0;
            for (size_t i = 1; i < contador_destinos_.size(); i++) {
                if (contador_destinos_[i] < contador_destinos_[min_index]) {
                    min_index = i;
                }
            }
            return min_index;
        }

        std::vector<atc_sim_ros2::msg::Waypoint> generateIntermediateWaypoints( const atc_sim_ros2::msg::Waypoint& start, const atc_sim_ros2::msg::Waypoint& end, int num_points) {
            std::vector<atc_sim_ros2::msg::Waypoint> intermediate_waypoints;

            double step_x = (end.x - start.x) / (num_points + 1);
            double step_y = (end.y - start.y) / (num_points + 1);
            double step_z = (end.z - start.z) / (num_points + 1);

            for (int i = 1; i <= num_points; i++) {
                atc_sim_ros2::msg::Waypoint wp;
                wp.x = start.x + step_x * i;
                wp.y = start.y + step_y * i;
                wp.z = start.z + step_z * i;
                intermediate_waypoints.push_back(wp);

                RCLCPP_INFO( rclcpp::get_logger("avion_logger"), "waypoint intermedio %d: {%.2f, %.2f, %.2f}", i, wp.x, wp.y, wp.z);
            }
            
            return intermediate_waypoints;
        }

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