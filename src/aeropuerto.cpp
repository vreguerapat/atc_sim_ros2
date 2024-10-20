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
           
            update_timer_ = this->create_wall_timer( 1s, [this]() { update_airport(0.01); });

            avion_timer_= this->create_wall_timer(30s, [this]() {agregarAvion();});
        }

        //Funcion para agregar nuevo avion
        void agregarAvion()
        {
            Avion nuevo_avion;
            // Waypoints disponibles
            std::vector<std::array<float, 3>> waypoints = {
                {0.0, 0.0, 5.0},
                {5.0, 5.0, 5.0}
            };

            // Se le asigna waypoint aleatorio
            nuevo_avion.selectRandomWaypoint(waypoints);
            lista_aviones_.push_back(nuevo_avion);
            RCLCPP_INFO(this->get_logger(), "Se agregó un nuevo avion");
        }
    
    private:
        //Variables privadas
        rclcpp::Publisher<atc_sim_ros2::msg::ListaAviones>::SharedPtr lista_aviones_publisher_;
        rclcpp::TimerBase::SharedPtr update_timer_;
        rclcpp::TimerBase::SharedPtr avion_timer_;
        std::vector<Avion> lista_aviones_;
        // Vector de waypoints, cada uno con coord. x y z
        std::vector<std::array<float,3>> waypoints_ = {
            {0.0, 0.0, 5.0},
            {5.0, 5.0, 5.0}
        };

        void update_airport(double delta_time)
        {
            // Coordenadas del waypoint
            double waypoint_x = waypoints_[0][0];
            double waypoint_y = waypoints_[0][1];
            double waypoint_z = waypoints_[0][2];
            //Hay que crear el mensaje del nuevo avion
            atc_sim_ros2::msg::ListaAviones msg_lista;

            for(auto &avion : lista_aviones_){
                avion.update(delta_time);

                atc_sim_ros2::msg::Flight avion_msg;
                avion_msg.id = avion.getID();
                avion_msg.airline = avion.getAirline();
                avion_msg.posx = avion.getPosX();
                avion_msg.posy = avion.getPosY();
                avion_msg.posz = avion.getPosZ();
                avion_msg.speed = avion.getSpeed();
                avion_msg.bearing = avion.getBearing();

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

        
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto aeropuerto = std::make_shared<Aeropuerto>();
    rclcpp::spin(aeropuerto);
    rclcpp::shutdown();
    return 0;
}