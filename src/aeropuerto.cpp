#include "atc_sim_ros2/aeropuerto.hpp"
#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include "atc_sim_ros2/avion.hpp"
#include "atc_sim_ros2/msg/waypoint_update.hpp"
#include <vector>
#include <chrono>
#include <memory>
#include <cmath>


using namespace std::chrono_literals;

Aeropuerto::Aeropuerto() : Node("aeropuerto")
{
      //Se publica el mensaje lista_aviones
    lista_aviones_publisher_ = this->create_publisher<atc_sim_ros2::msg::ListaAviones>("lista_aviones",10);

    waypoints_sub_ = this->create_subscription<atc_sim_ros2::msg::WaypointUpdate>("waypoint_update", 10, std::bind(&Aeropuerto::updateWaypoints, this, std::placeholders::_1));
           
    update_timer_ = this->create_wall_timer( 1s, [this]() { update_airport(0.01); });

    avion_timer_= this->create_wall_timer(10s, [this]() {agregarAvion();});
        
}

void Aeropuerto::updateWaypoints(const atc_sim_ros2::msg::WaypointUpdate& waypoint_update) {
    RCLCPP_INFO(this->get_logger(), "Recibiendo waypoints para el avion %s", waypoint_update.avion_id.c_str());
    for (auto& avion : lista_aviones_) {
        if (avion.getID() == waypoint_update.avion_id) {
            avion.clearWaypoints();
            avion.addWaypoints(waypoint_update.waypoints);
            avion.setSpeed(waypoint_update.speed);
            RCLCPP_INFO(this->get_logger(), "Waypoints y velocidad actualizados para el avion %s", avion.getID().c_str());
        }
    }
}

void Aeropuerto::agregarAvion()
{
    Avion nuevo_avion;

    bool position_found = false;
    int attempts = 0;
    while (!position_found && attempts < 10) {
        // Se genera una nueva posicion aleatoria para el avion
        double new_pos_x = generateRandomCoordinate();
        double new_pos_y = generateRandomCoordinate();
        double new_pos_z = generateRandomAltitude();

        nuevo_avion.setPosX(new_pos_x);
        nuevo_avion.setPosY(new_pos_y);
        nuevo_avion.setPosZ(new_pos_z);

        // Se va a verificar que el nuevo avion no este demasiado cerca de los aviones ya existentes
        bool too_close = false;
        for (const auto& avion : lista_aviones_) {
            double distance = calculateDistance(nuevo_avion, avion);
            if (distance < 6) {
                too_close = true;
                break;
            }
        }

        // Si esta sufiecientemente lejos se asigna la posicion y se agrega
        if (!too_close) {
            position_found = true;
            RCLCPP_INFO(this->get_logger(), "Se agregó un nuevo avion");
            lista_aviones_.push_back(nuevo_avion);
        } else {
            // Si sigue estando demasiado cerca se intenta de nuevo
            attempts++;
            RCLCPP_WARN(this->get_logger(), "Posicion no segura, intentando nuevamente...");
        }

    }

    // Si no se encontró una posicion valida en los 10 intentos no se agrega el avion
    if (!position_found) {
        RCLCPP_WARN(this->get_logger(), "No se pudo encontrar una posicion segura");
    }
    
}

double Aeropuerto::generateRandomCoordinate() {
    double random_value = static_cast<double>(rand() % 1001);
    if (rand() % 2 == 0) {
        return 19.0 + random_value / 1000.0; // Rango [19,20]
    } else {
        return -20.0 - random_value / 1000.0; // Rango [-20,-19]
    }
}

double Aeropuerto::generateRandomAltitude() {
    double random_value = static_cast<double>(rand() % 5001);
    return random_value / 1000.0; // Rango [0,5]
}

double Aeropuerto::calculateDistance(const Avion& avion1, const Avion& avion2) {
    double dx = avion1.getPosX() - avion2.getPosX();
    double dy = avion1.getPosY() - avion2.getPosY();
    double dz = avion1.getPosZ() - avion2.getPosZ();
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    return distance;
}

void Aeropuerto::update_airport(double delta_time)
{
    //Hay que crear el mensaje del nuevo avion
    atc_sim_ros2::msg::ListaAviones msg_lista;

    // Eliminar aviones con ruta completada
    lista_aviones_.erase(std::remove_if(lista_aviones_.begin(), lista_aviones_.end(), [](const Avion& avion) { return avion.getRutaCompletada(); }), lista_aviones_.end());

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
        avion_msg.ruta_completada = avion.getRutaCompletada();
        
        //Se agregan los waypoints al mensaje del avion
        for (const auto& wp : avion.getWaypoints()) {
            atc_sim_ros2::msg::Waypoint waypoint_msg;
            waypoint_msg.x = wp.x;
            waypoint_msg.y = wp.y;
            waypoint_msg.z = wp.z;
            avion_msg.waypoints.push_back(waypoint_msg);
        }

        msg_lista.aviones.push_back(avion_msg);

        std::cout << "ID: " << avion_msg.id
                    << ", Waypoints: " << avion_msg.waypoints.size()
                    << ", Ruta completada: " << avion_msg.ruta_completada 
                    << std::endl;  
    }

    lista_aviones_publisher_->publish(msg_lista);
}

