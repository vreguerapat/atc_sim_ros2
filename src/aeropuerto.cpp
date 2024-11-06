#include "atc_sim_ros2/aeropuerto.hpp"
#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include "atc_sim_ros2/avion.hpp"
#include <vector>
#include <chrono>
#include <memory>


using namespace std::chrono_literals;

Aeropuerto::Aeropuerto() : Node("aeropuerto")
{
      //Se publica el mensaje lista_aviones
    lista_aviones_publisher_ = this->create_publisher<atc_sim_ros2::msg::ListaAviones>("lista_aviones",10);
           
    update_timer_ = this->create_wall_timer( 1s, [this]() { update_airport(0.01); });

    avion_timer_= this->create_wall_timer(30s, [this]() {agregarAvion();});
        
}


//Funcion para agregar nuevo avion
void Aeropuerto::agregarAvion()
{
    Avion nuevo_avion;

    RCLCPP_INFO(this->get_logger(), "Se agregó un nuevo avion");
    
    // Posicion inicial del avion
    atc_sim_ros2::msg::Waypoint start;
    start.x = nuevo_avion.getPosX();
    start.y = nuevo_avion.getPosY();
    start.z = nuevo_avion.getPosZ();

    // Posicion punto final, fijo
    atc_sim_ros2::msg::Waypoint end;
    end.x = 5.0;
    end.y = 5.0;
    end.z = 5.0;

    // Generar 5 puntos intermedios
    std::vector<atc_sim_ros2::msg::Waypoint> waypoints = nuevo_avion.generateIntermediateWaypoints(start, end, 2);
    nuevo_avion.addWaypoints(waypoints);

    lista_aviones_.push_back(nuevo_avion);
    
}

void Aeropuerto::update_airport(double delta_time)
{
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
                  << ", PosX: " << avion_msg.posx
                    << ", PosY: " << avion_msg.posy
                    << ", PosZ: " << avion_msg.posz
                    << std::endl;  
    }

    lista_aviones_publisher_->publish(msg_lista);
}

