#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include "avion.hpp"
#include <vector>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class Aeropuerto : public rclcpp::Node
{
    public:
        Aeropuerto() : Node("aeropuerto")
        {
            //Se publica el mensaje lista_aviones
            lista_aviones_publisher_ = this->create_publisher<atc_sim_ros2::msg::ListaAviones>("lista_aviones",10);

            //Agregar avion cada 10 segundos
            timer_ = this->create_wall_timer( 10s, [this]() { agregarAvion(); });
        }
    
    private:
        //Funcion para agregar nuevo avion
        void agregarAvion()
        {
            Avion nuevo_avion;
            lista_aviones_.push_back(nuevo_avion);

            //Hay que crear el mensaje del nuevo avion
            atc_sim_ros2::msg::ListaAviones msg_lista;

            RCLCPP_INFO(this->get_logger(), "Publicando lista de %zu aviones", lista_aviones_.size());
            
            for (const auto &avion : lista_aviones_){
                atc_sim_ros2::msg::Flight avion_msg;
                avion_msg.id = avion.getID();
                avion_msg.airline = avion.getAirline();
                avion_msg.posx = avion.getPosX();
                avion_msg.posy = avion.getPosY();
                avion_msg.posz = avion.getPosZ();
                avion_msg.speed =avion.getSpeed();

                msg_lista.aviones.push_back(avion_msg);

                std::cout << "ID: " << avion_msg.id
                          << ", Airline: " << avion_msg.airline
                          << ", PosX: " << avion_msg.posx
                          << ", PosY: " << avion_msg.posy
                          << ", PosZ: " << avion_msg.posz
                          << ", Speed: " << avion_msg.speed
                          << std::endl;
            }

            
            lista_aviones_publisher_->publish(msg_lista);
        }

        //Variables privadas
        rclcpp::Publisher<atc_sim_ros2::msg::ListaAviones>::SharedPtr lista_aviones_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<Avion> lista_aviones_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto aeropuerto = std::make_shared<Aeropuerto>();
    rclcpp::spin(aeropuerto);
    rclcpp::shutdown();
    return 0;
}