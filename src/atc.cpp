
#include "atc_sim_ros2/atc.hpp"
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

ATC::ATC() : Node("atc"), contador_colisiones_(0) {
    aviones_sub_ = this->create_subscription<atc_sim_ros2::msg::ListaAviones>( "lista_aviones", 10, std::bind(&ATC::avionesCallback, this, std::placeholders::_1));

    waypoints_pub_ = this->create_publisher<atc_sim_ros2::msg::WaypointUpdate>("waypoint_update", 10);

    // Se definen los destinos, los mismos que los waypoints de visualizador.cpp
    atc_sim_ros2::msg::Waypoint destino1, destino2, destino3;
    destino1.x = 5.0; destino1.y = 5.0; destino1.z = 5.0;
    destino2.x = 0.0; destino2.y = 0.0; destino2.z = 5.0;
    destino3.x = -6.0; destino3.y = -4.0; destino3.z = 7.0;

    waypoints_destino_ = {destino1, destino2};
    contador_destinos_ =  {0,0};     

    collision_check_timer_ = this->create_wall_timer(
        1s, [this]() {
            if (!aviones_lista_) {
                return;
            }
            checkCollisions(aviones_lista_); //Simular colisiones con 5 segundos de anticipacion
        }
    );                                                                                                                                                                                                                               
}

void ATC::avionesCallback(const atc_sim_ros2::msg::ListaAviones::SharedPtr msg) {
    if (!msg) {
        return;
    }
    aviones_lista_ = msg;
    manageRoutes(msg);
} 

void ATC::manageRoutes(const atc_sim_ros2::msg::ListaAviones::SharedPtr lista_aviones) {
    for (auto& avion_msg : lista_aviones->aviones) {             
        // Se verifica si el avion ha completado su ruta
        if (!avion_msg.ruta_completada && avion_msg.waypoints.empty()) {
            //RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Avion %s tiene %zu waypoints antes de asignar ruta", avion_msg.id.c_str(), avion_msg.waypoints.size());
            // Se le asigna destino
            assignRoute(avion_msg);
        } else if (avion_msg.ruta_completada) {
            //RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "La ruta del avion %s ya está completada", avion_msg.id.c_str());
        }
    }

}
        
void ATC::assignRoute (atc_sim_ros2::msg::Flight& avion_msg) {
    std::vector<atc_sim_ros2::msg::Waypoint> waypoints;

    if (avion_msg.ruta_completada && avion_msg.waypoints.empty()) {
        return;
    }
            
    atc_sim_ros2::msg::Waypoint start;
    start.x = avion_msg.posx;
    start.y = avion_msg.posy;
    start.z = avion_msg.posz;

    // Se selecciona el destino menos asignado
    int destino_index = selectLeastAssignedDestination();
    atc_sim_ros2::msg::Waypoint destino = waypoints_destino_[destino_index];

    // Se generan los waypoints intermedios
    waypoints = generateIntermediateWaypoints(start, destino, 2);

    // Si el avion ya tiene waypoints asignados no se asignan mas
    if (avion_msg.waypoints.empty()) {
        waypoints.push_back(destino);
        avion_msg.waypoints = waypoints;
        //RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Publicando waypoints para el avion %s", avion_msg.id.c_str());

        atc_sim_ros2::msg::WaypointUpdate update_msg;
       update_msg.avion_id = avion_msg.id;
        update_msg.waypoints = waypoints;
        update_msg.speed = avion_msg.speed;
        waypoints_pub_->publish(update_msg);
        contador_destinos_[destino_index]++; // Se actualiza contador
    }
            
}

bool ATC::willCollidePredictive(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2, double time_ahead, double threshold_distance) {
    double delta_time = 0.01;
    double climb_rate = 5.0 * delta_time;
    double dx1 = avion1.speed * std::cos(avion1.bearing);
    double dy1 = avion1.speed * std::sin(avion1.bearing);
    double dz1 = climb_rate;

    double dx2 = avion2.speed * std::cos(avion2.bearing);
    double dy2 = avion2.speed * std::sin(avion2.bearing);
    double dz2 = climb_rate;

    double future_x1 = avion1.posx + dx1 * time_ahead;
    double future_y1 = avion1.posy + dy1 * time_ahead;
    double future_z1 = avion1.posz + dz1 * time_ahead;

    double future_x2 = avion2.posx + dx2 * time_ahead;
    double future_y2 = avion2.posy + dy2 * time_ahead;
    double future_z2 = avion2.posz + dz2 * time_ahead;

    double distance = std::sqrt(
        std::pow(future_x1 - future_x2, 2) +
        std::pow(future_y1 - future_y2, 2) +
        std::pow(future_z1 - future_z2, 2)
    );
    RCLCPP_INFO(this->get_logger(), "Distancia entre %s y %s: %.2f", avion1.id.c_str(), avion2.id.c_str(), distance);
    return distance < threshold_distance;
}

bool ATC::checkRouteCollision(const atc_sim_ros2::msg::Flight& avion_msg, const std::vector<atc_sim_ros2::msg::Waypoint>& new_route) { 
    // Se comprueba distancia entre la ruta propuesta y las rutas de otros aviones 
    for (const auto& wp : new_route) { 
        for (const auto& otro_avion : aviones_lista_->aviones) { 
            double distance = calculateDistance(avion_msg, otro_avion); 
            if (distance < 5.0) { 
                return true; // Hay colision 
            } 
        } 
    } 
    
    return false; // No hay colision 
}

void ATC::adjustRoute(atc_sim_ros2::msg::Flight& avion_msg) { 
    // Para ajustar la ruta se puede cambiar el destino o mejor añadir más waypoints para alejar el avion de los otros. 
    // Para cambiar el destino sería: 
    int destino_index = selectLeastAssignedDestination(); 
    atc_sim_ros2::msg::Waypoint new_destino = waypoints_destino_[(destino_index + 1) % waypoints_destino_.size()]; 
    atc_sim_ros2::msg::Waypoint start; 
    start.x = avion_msg.posx; 
    start.y = avion_msg.posy; 
    start.z = avion_msg.posz; 
    
    std::vector<atc_sim_ros2::msg::Waypoint> new_waypoints = generateIntermediateWaypoints(start, new_destino, 2); 
    avion_msg.waypoints = new_waypoints; 
    atc_sim_ros2::msg::WaypointUpdate update_msg; 
    update_msg.avion_id = avion_msg.id; 
    update_msg.waypoints = new_waypoints; 
    update_msg.speed = avion_msg.speed;
    waypoints_pub_->publish(update_msg); RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Ruta ajustada para el avion %s", avion_msg.id.c_str()); 
}

int ATC::selectLeastAssignedDestination() {
    int min_index = 0;
    for (size_t i = 1; i < contador_destinos_.size(); i++) {
        if (contador_destinos_[i] < contador_destinos_[min_index]) {
            min_index = i;
        }
    }
    return min_index;
}

std::vector<atc_sim_ros2::msg::Waypoint> ATC::generateIntermediateWaypoints( const atc_sim_ros2::msg::Waypoint& start, const atc_sim_ros2::msg::Waypoint& end, int num_points) {
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

        //RCLCPP_INFO( rclcpp::get_logger("avion_logger"), "waypoint intermedio %d: {%.2f, %.2f, %.2f}", i, wp.x, wp.y, wp.z);
    }
            
    return intermediate_waypoints;
}

void ATC::checkCollisions(atc_sim_ros2::msg::ListaAviones::SharedPtr msg) {
    auto& aviones = msg->aviones;

    for (size_t i = 0; i < aviones.size(); i++) {
        for (size_t j = i+1; j < aviones.size(); j++) {
            /*if (willCollidePredictive(aviones[i], aviones[j], 2.0, 2.0)) {
                RCLCPP_WARN(this->get_logger(), "Posible colision predictiva entre %s y %s", aviones[i].id.c_str(), aviones[j].id.c_str());
                adjustTrajectory(aviones[i], aviones[j]);
            }*/
           double distance = calculateDistance(aviones[i], aviones[j]);
           // Verifica colisiones basadas en las posiciones actuales
           if (areTooClose(aviones[i], aviones[j], 5.0)) {
            RCLCPP_WARN(this->get_logger(), "Posible colision entre %s y %s. Distancia: %.2f", aviones[i].id.c_str(), aviones[j].id.c_str(), distance);
            adjustTrajectory(aviones[i], aviones[j]);
           }

           //Verifica colisiones en las rutas asignadas
           if (areRoutesIntersecting(aviones[i], aviones[j])) {
            //RCLCPP_WARN(this->get_logger(), "Rutas en conflicto entre %s y %s", aviones[i].id.c_str(), aviones[j].id.c_str());
            //adjustTrajectory(aviones[i], aviones[j]);
           }

            double distancia = calculateDistance(aviones[i], aviones[j]);
           if (distancia < 1) {
            RCLCPP_ERROR(this->get_logger(), "Colsion entre %s y %s", aviones[i].id.c_str(), aviones[j].id.c_str());
            contador_colisiones_++;
           }
        }
    }
    // Publica el total de colisiones detectadas
    RCLCPP_INFO(this->get_logger(), "Total colisiones detectadas: %d", contador_colisiones_);
}

bool ATC::areTooClose(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2, double threshold_distance) {
    double dx = avion1.posx - avion2.posx;
    double dy = avion1.posy - avion2.posy;
    double dz = avion1.posz - avion2.posz;

    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    RCLCPP_DEBUG(this->get_logger(), "Distancia actual entre %s y %s: %.2f", avion1.id.c_str(), avion2.id.c_str(), distance);

    return distance < threshold_distance;
}

bool ATC::areRoutesIntersecting(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2) {
    // Umbral de tiempo seguro (en segundos)
    const double safe_time_threshold = 1.0; 

    for (const auto& wp1 : avion1.waypoints) {
        for (const auto& wp2 : avion2.waypoints) {
            if (std::abs(wp1.x - wp2.x) < 0.5 &&
                std::abs(wp1.y - wp2.y) < 0.5 &&
                std::abs(wp1.z - wp2.z) < 0.5) {

                    // Se calcula el tiempo de llagada para ambos aviones
                    double time_to_wp1 = calculateTimeToWaypoint(avion1, wp1);
                    double time_to_wp2 = calculateTimeToWaypoint(avion2, wp2);

                    //RCLCPP_INFO(this->get_logger(), "Interseccion detectada en {%.2f, %.2f, %.2f}. Tiempos: %.2fs y %.2fs", wp1.x, wp1.y, wp1.z, time_to_wp1, time_to_wp2);

                    // Evaluar si los tiempos son peligrosos
                    if (std::abs(time_to_wp1 - time_to_wp2) < safe_time_threshold) {
                        return true;
                    }
                }
        }
    }
    return false;
}

double ATC::calculateTimeToWaypoint(const atc_sim_ros2::msg::Flight& avion, const atc_sim_ros2::msg::Waypoint& wp) {
    double dx = wp.x - avion.posx;
    double dy = wp.y - avion.posy;
    double dz = wp.z - avion.posz;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    return distance/avion.speed;
}

void ATC::adjustTrajectory(atc_sim_ros2::msg::Flight& avion1, atc_sim_ros2::msg::Flight& avion2) {
    double distance = calculateDistance(avion1, avion2);

    if (distance < 5.0) {
        // 1. Ajustar velocidad
        adjustSpeed(avion1, avion2);

        // 2. Si estan muy cerca en altura, ajustar altitud
        /*
        if (abs(avion1.posz - avion2.posz) < 2) {
        adjust Altitud(avion1);
        }
        */

       // 3. Modificar ruta del avion
       /* 
       else if (avion1.waypoints.size() > 1) {
       adjustNextWaypoint(avion1);
       }*/
        
    }
    RCLCPP_INFO(this->get_logger(), "Dist > 5, no se hacen cambios");
}

void ATC::adjustAltitud(atc_sim_ros2::msg::Flight& avion_msg) {
    if (avion_msg.waypoints.empty()) {
        return;
    }

    // Se define un limite máximo de altitud
    double max_altitude = 10.0;

    RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Ajustando altitud del avion %s para evitar colision", avion_msg.id.c_str()); 
    // Para ello se modifica la pos z del siguiente waypoint
    double altitude_change = 5.0;

    // Limitar la altitud para que no supere el maximo
    if (avion_msg.waypoints.front().z + altitude_change > max_altitude) {
        avion_msg.waypoints.front().z = max_altitude;
    } else {
        avion_msg.waypoints.front().z += altitude_change; 
    }
    
    // Se publica la actualizacion del waypoint
    atc_sim_ros2::msg::WaypointUpdate update_msg;
    update_msg.avion_id = avion_msg.id;
    update_msg.waypoints = avion_msg.waypoints;
    update_msg.speed = avion_msg.speed;
    waypoints_pub_->publish(update_msg);

    RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Altitud ajustada para el primer waypoint del avion %s. Nueva altitud: %.2f", avion_msg.id.c_str(), avion_msg.waypoints.front().z);
}

void ATC::adjustNextWaypoint(atc_sim_ros2::msg::Flight& avion_msg) {
    if (avion_msg.waypoints.empty()) {
        return;
    }
    // Se cambia el siguient waypoint
    atc_sim_ros2::msg::Waypoint new_wp = avion_msg.waypoints[0];
    new_wp.x += 1.0;
    new_wp.y += 1.0;

    // Se actualizan los waypoints del avion
    avion_msg.waypoints[0] = new_wp;

    // Se publican las actualizaciones de la ruta
    atc_sim_ros2::msg::WaypointUpdate update_msg;
    update_msg.avion_id = avion_msg.id;
    update_msg.waypoints.push_back(new_wp);
    update_msg.speed = avion_msg.speed;
    waypoints_pub_->publish(update_msg);

    RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Ajustando waypoint del avion %s para evitar colision", avion_msg.id.c_str());
}   

void ATC::adjustSpeed(atc_sim_ros2::msg::Flight& avion1, atc_sim_ros2::msg::Flight& avion2) {
    RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Ajustando velocidades de %s y %s para evitar colision", avion1.id.c_str(), avion2.id.c_str());
    // Parámetros configurables
    const double min_speed = 5.0;
    const double max_speed = 30.0;
    const double speed_adjustment = 5.0;

    // Calcular diferencias angulares
    double angle_diff1 = normalizeAngle(calculateBearing(avion1, avion2) - avion1.bearing);
    double angle_diff2 = normalizeAngle(calculateBearing(avion2, avion1) - avion2.bearing);

    // Diferencia de rumbos, para saber si avanzan en direcciones similares
    double bearing_diff = normalizeAngle(avion1.bearing - avion2. bearing);

    // Configuracion valida, uno detrás del otro:
    // 1. Uno de los aviones debe estar en el cono frontal del otro
    // 2. Sus rumbos deben ser parecidos, +- 30 grados
    bool one_behind_other = 
        (std::abs(angle_diff1) < M_PI_2) && // Avion 1 en cono frontal de Avion 2
        (std::abs(angle_diff2) > M_PI_2) && // Avion 2 fuera del cono frontal de Avion 1
        (std::abs(bearing_diff) < M_PI / 6); // Rumbos compatibles
    

    
    if (!one_behind_other) {
        RCLCPP_INFO(this->get_logger(), "Los aviones no están en configuración válida");
        return;
    }

    if (one_behind_other) {
        // Determinar quien acelera y quien frena
        if (std::abs(angle_diff1) < M_PI_2) {
            // Avion 1 está delante, acelera
            avion1.speed = std::min(avion1.speed + speed_adjustment, max_speed);
            avion2.speed = std::max(avion2.speed - speed_adjustment, min_speed);
        } else {
            // Avion 2 está delante, acelera
            avion2.speed = std::min(avion2.speed + speed_adjustment, max_speed);
            avion1.speed = std::max(avion1.speed - speed_adjustment, min_speed);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Ajustando velocidades:");
    RCLCPP_INFO(this->get_logger(), "Avion %s - Velocidad: %.2f", avion1.id.c_str(), avion1.speed);
    RCLCPP_INFO(this->get_logger(), "Avion %s - Velocidad: %.2f", avion2.id.c_str(), avion2.speed);
    

    // Se publica la nueva velocidad
    atc_sim_ros2::msg::WaypointUpdate update_msg1, update_msg2;
    
    update_msg1.avion_id = avion1.id;
    update_msg1.waypoints = avion1.waypoints;
    update_msg1.speed = avion1.speed;

    update_msg2.avion_id = avion2.id;
    update_msg2.waypoints = avion2.waypoints;
    update_msg2.speed = avion2.speed;

    waypoints_pub_->publish(update_msg1);
    waypoints_pub_->publish(update_msg2);

    RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Velocidades ajustadas: %s (%.2f) y %s (%.2f)", avion1.id.c_str(), avion1.speed, avion2.id.c_str(), avion2.speed);
}

double ATC::calculateBearing(atc_sim_ros2::msg::Flight& from, atc_sim_ros2::msg::Flight& to) {
    double delta_x = to.posx - from.posx;
    double delta_y = to.posy - from.posy;

    double bearing = std::atan2(delta_y, delta_x);

    if (bearing < 0) {
        bearing += 2 * M_PI;
    }

    return bearing;
}

double ATC::normalizeAngle(double angle){
    angle = std::fmod(angle, 2 * M_PI);

    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle;
}

double ATC::calculateDistance(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2) {
    double dx = avion1.posx - avion2.posx;
    double dy = avion1.posy - avion2.posy;
    double dz = avion1.posz - avion2.posz;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    return distance;
}
