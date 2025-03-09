#include "atc_sim_ros2/aeropuerto.hpp"
#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include "atc_sim_ros2/avion.hpp"
#include "atc_sim_ros2/msg/waypoint_update.hpp"
#include <vector>
#include <chrono>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <ctime>


using namespace std::chrono_literals;

Aeropuerto::Aeropuerto() : Node("aeropuerto")
{
      //Se publica el mensaje lista_aviones
    lista_aviones_publisher_ = this->create_publisher<atc_sim_ros2::msg::ListaAviones>("lista_aviones",10);

    waypoints_sub_ = this->create_subscription<atc_sim_ros2::msg::WaypointUpdate>("waypoint_update", 10, std::bind(&Aeropuerto::updateWaypoints, this, std::placeholders::_1));
           
    update_timer_ = this->create_wall_timer( 1s, [this]() { update_airport(0.01); });

    avion_timer_= this->create_wall_timer(30s, [this]() {agregarAvion();}); 

    std::cout << "==========================\n";
    std::cout << "      NODO AEROPUERTO     \n";
    std::cout << "==========================\n";
    std::cout << "Nodo Aeropuerto iniciado correctamente.\n";
    std::cout << "Esperando la creacón de aviones...\n";
        
}

Aeropuerto::~Aeropuerto() {
    RCLCPP_INFO(this->get_logger(), "El nodo Aeropuerto ha finalizado.");
    RCLCPP_INFO(this->get_logger(), "Número total de aviones simulados: %d", aviones_totales);
}

void Aeropuerto::updateWaypoints(const atc_sim_ros2::msg::WaypointUpdate& waypoint_update) {
    
    // Primero se verifica si la lista de waypoints está vacía para eliminar el avion
    if (waypoint_update.waypoints.empty()) {
        lista_aviones_.erase(
            std::remove_if(
                lista_aviones_.begin(), 
                lista_aviones_.end(), 
                [&waypoint_update](const Avion& avion) {
                    return avion.getID() == waypoint_update.avion_id;
                }
            ), lista_aviones_.end()
        );
    RCLCPP_INFO(this->get_logger(), "%s ha aterrizado con éxito.", waypoint_update.avion_id.c_str());
    //RCLCPP_ERROR(this->get_logger(), "Avion %s eliminado de la lista de aviones", waypoint_update.avion_id.c_str());
    } else {
        for (auto& avion : lista_aviones_) {
        if (avion.getID() == waypoint_update.avion_id) {
            avion.clearWaypoints();
            avion.addWaypoints(waypoint_update.waypoints);
            avion.setSpeed(waypoint_update.speed);
            RCLCPP_INFO(this->get_logger(), "Actualizando información de %s...", waypoint_update.avion_id.c_str());
        }
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
        double new_pos_x = generateRandomCoordinateX();
        double new_pos_y = generateRandomCoordinateY();
        double new_pos_z = generateRandomAltitude();

        nuevo_avion.setPosX(new_pos_x);
        nuevo_avion.setPosY(new_pos_y);
        nuevo_avion.setPosZ(new_pos_z);

        // Se va a verificar que el nuevo avion no este demasiado cerca de los aviones ya existentes
        bool too_close = false;
        for (const auto& avion : lista_aviones_) {
            double distance = calculateDistance(nuevo_avion, avion);
            if (distance < 7) {
                too_close = true;
                break;
            }
        }

        // Si esta sufiecientemente lejos se asigna la posicion y se agrega
        if (!too_close) {
            position_found = true;
            // Bearing hacia las coord (28, 20, 5)
            double target_x = 28.0;
            double target_y = 20.0;
            double bearing = std::atan2(target_y - new_pos_y, target_x - new_pos_x);
            nuevo_avion.setBearing(bearing);
            RCLCPP_INFO(this->get_logger(), "Se agregó un nuevo avion, %s", nuevo_avion.getID().c_str());
            lista_aviones_.push_back(nuevo_avion);
            aviones_totales += 1;
            RCLCPP_INFO(this->get_logger(), "Aviones en el espacio aéreo: %zu", lista_aviones_.size());
            RCLCPP_INFO(this->get_logger(), "Histórico de aviones en la simulación: %d", aviones_totales);
        } else {
            // Si sigue estando demasiado cerca se intenta de nuevo
            attempts++;
            RCLCPP_WARN(this->get_logger(), "Posicion no segura, intentando nuevamente...");
        }

    }

    // Si no se encontró una posicion valida en los 10 intentos no se agrega el avion
    if (!position_found) {
        RCLCPP_WARN(this->get_logger(), "No se pudo encontrar una posicion segura. Conflicto con otro avión en posición inicial.\n");
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

double Aeropuerto::generateRandomCoordinateX() {
    lado = rand() % 3; // Selecciona un lado (0,1,2)

    switch (lado) {
        case 0: // Lado 1 (vertical positivo)
            return 45.0 + static_cast<double>(rand() % 6); // Rango [45, 50]
        case 1: // Lado 2 (horizontal superior)
            return -50.0 + static_cast<double>(rand() % 101); // Rango [-50, 50]
        case 2: // Lado 3 (horizontal inferior)
            if (rand() % 2 == 0) {
                return -50.0 - static_cast<double>(rand() % 41); // Rango [-50, -10]
            } else {
                return 10.0 + static_cast<double>(rand() % 41); // Rango [10, 50]
            }
        case 3: // Lado 4 (vertical negativo)
            return -45.0 - static_cast<double>(rand() % 6); // Rango [-45, -50]
        default:
            return 55.0; // Caso de error
    }
}

double Aeropuerto::generateRandomCoordinateY() {
    //int lado = rand() % 3; // Selecciona un lado (0,1,2)

    switch (lado) {
        case 0: // Lado 1 (vertical)
            return static_cast<double>(rand() % 41); // Rango [0, 40]
        case 1: // Lado 2 (horizontal superior)
            return 40.0 + static_cast<double>(rand() % 6); // Rango [40, 45] 
        case 2: // Lado 3 (horizontal inferior)
            return -5.0 + static_cast<double>(rand() % 6); // Rango [-5, 0] HACERLO [-10, -5]
        case 3: // Lado 4 (vertical negativo)
            return static_cast<double>(rand() % 41); // Rango [0, 40]
        default: 
            return 55.0; // Caso de error
        
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

        /*std::cout << "ID: " << avion_msg.id
                    << ", Waypoints: " << avion_msg.waypoints.size()
                    << ", Ruta completada: " << avion_msg.ruta_completada 
                    << std::endl;  */
    }

    lista_aviones_publisher_->publish(msg_lista);
}

