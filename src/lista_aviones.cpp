#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/flight.hpp"

#include <vector>
#include <string>

using namespace std::chrono_literals;

class ListaAviones : public rclcpp::Node
{
    public:
        ListaAviones() : Node("lista_aviones")
        {
            // Creo suscriptores al mismo topic donde publican los mensajes los nodos avion
            for (int i = 1; i <= 3; i++){
                std::string topic_name = "/avion_" + std::to_string(i) +"/flight_info";
                auto callback = [this](const atc_sim_ros2::msg::Flight::SharedPtr msg) {
                    this->updateListaAviones(*msg);
                };

                subscriptores_.push_back(this->create_subscription<atc_sim_ros2::msg::Flight>("flight_topic", 10, callback));
            }

            //Imprime la lista
            auto publish_ListaAviones = [this]() -> void {
                RCLCPP_INFO(this->get_logger(), "Lista de aviones actualizada:");
                for (const auto &avion : lista_aviones_) {
                    std::cout <<"Avion ID: " << avion.id <<", Airline: " << avion.airline << std::endl;
                }
            };

            timer_ = this->create_wall_timer( 5s, publish_ListaAviones);
        }
    private:
        // Actualiza la lista con los nuevos datos. Si encuentra el mismo id solo actualiza los datos. Si no lo encuentra lo añade
        void updateListaAviones(const atc_sim_ros2::msg::Flight &msg)
        {
            bool found = false;
            for (auto &avion : lista_aviones_) {
                if (avion.id == msg.id) {
                    avion = msg;
                    found = true;
                    break;
                }
            }
            if (!found) {
                lista_aviones_.push_back(msg);
            }
        }

        std::vector<atc_sim_ros2::msg::Flight> lista_aviones_;
        std::vector<rclcpp::Subscription<atc_sim_ros2::msg::Flight>::SharedPtr> subscriptores_;
        rclcpp::TimerBase::SharedPtr timer_;

        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ListaAviones>());
    rclcpp::shutdown();

    return 0;
}