
#include "atc_sim_ros2/atc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/flight.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include "atc_sim_ros2/avion.hpp"
#include "atc_sim_ros2/msg/waypoint.hpp"
#include "atc_sim_ros2/msg/waypoint_update.hpp"
#include <vector>
#include <cmath>
#include <optional>
#include <cstdlib>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

ATC::ATC() : Node("atc"), 
             contador_colisiones_(0),
             landing32L(false),
             landing32R(false)
              
{
    aviones_sub_ = this->create_subscription<atc_sim_ros2::msg::ListaAviones>( "lista_aviones", 10, std::bind(&ATC::avionesCallback, this, std::placeholders::_1));

    waypoints_pub_ = this->create_publisher<atc_sim_ros2::msg::WaypointUpdate>("waypoint_update", 10);

    
    // Se definen los destinos, los mismos que los waypoints de visualizador.cpp
    atc_sim_ros2::msg::Waypoint destino1, destino2, destino3;
    destino1.x = 5.0; destino1.y = 5.0; destino1.z = 5.0;
    destino2.x = 0.0; destino2.y = 0.0; destino2.z = 5.0;
    destino3.x = -6.0; destino3.y = -4.0; destino3.z = 7.0;

    atc_sim_ros2::msg::Waypoint wp1, wp2, wp3, wp4, wp5, wp6, wp7, wp8;
    wp1.x = 40.0; wp1.y = 20.0; wp1.z = 5.0;
    wp2.x = 37.0; wp2.y = 30.0; wp2.z = 5.0;
    wp3.x = 30.0; wp3.y = 35.0; wp3.z = 5.0;
    wp4.x = 20.0; wp4.y = 30.0; wp4.z = 5.0; // Wp salida
    wp5.x = 17.0; wp5.y = 20.0; wp5.z = 5.0;
    wp6.x = 20.0; wp6.y = 10.0; wp6.z = 5.0;
    wp7.x = 30.0; wp7.y = 5.0; wp7.z = 5.0;
    wp8.x = 37.0; wp8.y = 10.0; wp8.z = 5.0;
    circuit1_ = {wp1, wp2, wp3, wp4, wp5, wp6, wp7, wp8}; 

    waypoints_destino_ = {destino1, destino2};
    contador_destinos_ =  {0,0}; 

    atc_sim_ros2::msg::Waypoint wp9, wp10, wp11;
    wp9.x = 10.0; wp9.y = 25.0; wp9.z = 5.0;
    wp10.x = 5.0; wp10.y = 15.0; wp10.z = 5.0;
    wp11.x = 5.0; wp11.y = 0.0; wp11.z = 0.0;
    landing_route32L_ = {wp4, wp9, wp10, wp11};

    // Waypoints segundo circuito de espera
    atc_sim_ros2::msg::Waypoint wp12, wp13, wp14, wp15, wp16, wp17, wp18, wp19;
    wp12.x = -40.0; wp12.y = 20.0; wp12.z = 5.0;
    wp13.x = -37.0; wp13.y = 30.0; wp13.z = 5.0;
    wp14.x = -30.0; wp14.y = 35.0; wp14.z = 5.0;
    wp15.x = -20.0; wp15.y = 30.0; wp15.z = 5.0; //Wp salida
    wp16.x = -17.0; wp16.y = 20.0; wp16.z = 5.0;
    wp17.x = -20.0; wp17.y = 10.0; wp17.z = 5.0;
    wp18.x = -30.0; wp18.y = 5.0; wp18.z = 5.0;
    wp19.x = -37.0; wp19.y = 10.0; wp19.z = 5.0;
    circuit2_ = {wp12, wp13, wp14, wp15, wp16, wp17, wp18, wp19};

    // Segunda opcion landing
    atc_sim_ros2::msg::Waypoint wp20, wp21, wp22;
    wp20.x = -10.0; wp20.y = 25.0; wp20.z = 5.0;
    wp21.x = -5.0; wp21.y = 15.0; wp21.z = 5.0;
    wp22.x = -5.0; wp22.y = 0.0; wp22.z = 0.0;
    landing_route32R_ = {wp15, wp20, wp21, wp22};

   
    collision_check_timer_ = this->create_wall_timer(
        1s, [this]() {
            if (!aviones_lista_) {
                return;
            }
            checkCollisions(aviones_lista_); 
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
            // Se le asigna la ruta inicial
            assignRoute(avion_msg);
        } else if (avion_msg.waypoints.empty() && avion_msg.ruta_completada) {
            //landing = false; // El avion ha aterrizado
            //RCLCPP_INFO(this->get_logger(), "El avion %s ha aterrizado. Se permite el siguiente aterrizaje", avion_msg.id.c_str());
        } else if (!avion_msg.waypoints.empty()) {
            // Caso para verificar si avion en waypoint de salida
            //RCLCPP_INFO(this->get_logger(), "Valor de landing para pista 32L: %s", landing32L ? "true" : "false");
            //RCLCPP_INFO(this->get_logger(), "Valor de landing para pista 32R: %s", landing32R ? "true" : "false");
            //atc_sim_ros2::msg::Waypoint current_wp = avion_msg.waypoints.front();
            atc_sim_ros2::msg::Waypoint current_wp;
            current_wp.x = avion_msg.posx; current_wp.y = avion_msg.posy; current_wp.z = avion_msg.posz;
            // Se identifica en que circuito está
            const auto& selected_circuit = findCircuit(avion_msg);
            RCLCPP_INFO(this->get_logger(), "Circuito seleccionado avion %s: %s", avion_msg.id.c_str(), selected_circuit == circuit1_ ? "Circuito1" : selected_circuit == circuit2_ ? "Circuito 2": "Ninguno");
            // Se verifica si el avion está cerca del waypoint {10.0, 25.0, 5.0} o {-10.0, 25.0, 5.0ç para dar permiso de aterrizaje a otro avion
            atc_sim_ros2::msg::Waypoint landing_wp1, landing_wp2;
            landing_wp1.x = 10.0; landing_wp1.y = 25.0; landing_wp1.z = 5.0;
            landing_wp2.x = -10.0; landing_wp2.y = 25.0; landing_wp2.z = 5.0;
            
           atc_sim_ros2::msg::Waypoint next_wp = avion_msg.waypoints.front();
           if (next_wp == landing_wp1){
                double distance_to_landing_wp = distanceBetweenWaypoints(current_wp, landing_wp1);
                if (distance_to_landing_wp <= 1) {
                    landing32L = false;
                    RCLCPP_INFO(this->get_logger(), "El avion %s está en el waypoint de liberación de aterrizaje del circuito 1. Se permite el siguiente aterrizaje", avion_msg.id.c_str());
                    
                }
           }

           if (next_wp == landing_wp2){
                double distance_to_landing_wp = distanceBetweenWaypoints(current_wp, landing_wp2);
                if (distance_to_landing_wp <= 1) {
                    landing32R = false;
                    RCLCPP_INFO(this->get_logger(), "El avion %s está en el waypoint de liberación de aterrizaje del circuito 2. Se permite el siguiente aterrizaje", avion_msg.id.c_str());
                    
                }
           }
            
            // Se verifica si el avion está cerca del waypoint de salida
            atc_sim_ros2::msg::Waypoint exit_wp;
            if (selected_circuit == circuit1_) {
                exit_wp.x = 20.0; exit_wp.y = 30.0; exit_wp.z = 5.0;
            } else if (selected_circuit == circuit2_) {
                exit_wp.x = -20.0; exit_wp.y = 30.0; exit_wp.z = 5.0;
            }

            double distance_to_exit_wp = distanceBetweenWaypoints(current_wp, exit_wp);
            
            if (distance_to_exit_wp <= 3) {
                if (selected_circuit == circuit1_ && !landing32L) {
                    assignLandingRoute32L(avion_msg);
                    landing32L = true;
                    //RCLCPP_INFO(this->get_logger(), "Valor de landing para pista 32L: %s", landing32L ? "true" : "false");
                } else if (selected_circuit == circuit2_ && !landing32R) {
                    assignLandingRoute32R(avion_msg);
                    landing32R = true;
                    //RCLCPP_INFO(this->get_logger(), "Valor de landing para pista 32R: %s", landing32R ? "true" : "false");
                } else if ((selected_circuit == circuit1_ && !landing32L) || (selected_circuit == circuit2_ && !landing32R)) {
                    //RCLCPP_ERROR(this->get_logger(), "AQUIIIII");
                    assignRoute(avion_msg);
                }
            }

            
        }
    }

}

const std::vector<atc_sim_ros2::msg::Waypoint>& ATC::findCircuit(const atc_sim_ros2::msg::Flight& avion_msg) {
    for (const auto& avion_wp : avion_msg.waypoints) {
        // Se verifica si el wp mas cercano pertenece al circuito 1
        auto it_circuit1 = std::find_if(circuit1_.begin(), circuit1_.end(), [&avion_wp](const atc_sim_ros2::msg::Waypoint& wp) {
            return wp.x == avion_wp.x && wp.y == avion_wp.y && wp.z == avion_wp.z;
        });

        if (it_circuit1 != circuit1_.end()) {
            return circuit1_;
        }

        // Se verifica si el wp mas cercano pertenece al circuito 1
        auto it_circuit2 = std::find_if(circuit2_.begin(), circuit2_.end(), [&avion_wp](const atc_sim_ros2::msg::Waypoint& wp) {
            return wp.x == avion_wp.x && wp.y == avion_wp.y && wp.z == avion_wp.z;
        });

        if (it_circuit2 != circuit2_.end()) {
            return circuit2_;
        }

    }
    

    // Si no se encuentra ninguno de los circuitos
    RCLCPP_WARN(this->get_logger(), "El avion %s no pertenece a ningún circuito", avion_msg.id.c_str());
    static std::vector<atc_sim_ros2::msg::Waypoint> empty_circuit;
    return empty_circuit;
}

void ATC::assignLandingRoute32L(atc_sim_ros2::msg::Flight& avion_msg) {
    avion_msg.waypoints.clear();
    avion_msg.waypoints = landing_route32L_; 

    atc_sim_ros2::msg::WaypointUpdate update_msg;
    update_msg.avion_id = avion_msg.id;
    update_msg.waypoints = avion_msg.waypoints;
    update_msg.speed = 25.0; // AQUI HE PUESTO VELOCIDAD FIJA
    waypoints_pub_->publish(update_msg);

    //RCLCPP_INFO(this->get_logger(), "Ruta de aterrizaje asignada para el avion %s", avion_msg.id.c_str());
}   

void ATC::assignLandingRoute32R(atc_sim_ros2::msg::Flight& avion_msg) {
    avion_msg.waypoints.clear();
    avion_msg.waypoints = landing_route32R_; 

    atc_sim_ros2::msg::WaypointUpdate update_msg;
    update_msg.avion_id = avion_msg.id;
    update_msg.waypoints = avion_msg.waypoints;
    update_msg.speed = 25.0; // AQUI HE PUESTO VELOCIDAD FIJA
    waypoints_pub_->publish(update_msg);

    //RCLCPP_INFO(this->get_logger(), "Ruta de aterrizaje asignada para el avion %s", avion_msg.id.c_str());
}  

void ATC::assignRoute (atc_sim_ros2::msg::Flight& avion_msg) {
    
    if (avion_msg.ruta_completada && avion_msg.waypoints.empty()) {
        return;
    }
    // Se identifica el wp mas cercano del circuito de espera
    atc_sim_ros2::msg::Waypoint closest_waypoint = findClosestWaypoint(avion_msg.posx, avion_msg.posy, avion_msg.posz);
    // Se ha de determinar a que circuito pertenece el waypoint mas cercano
    const std::vector<atc_sim_ros2::msg::Waypoint>& selected_circuit = (std::find_if(circuit1_.begin(), circuit1_.end(), [&closest_waypoint](const atc_sim_ros2::msg::Waypoint& wp) {
        return wp.x == closest_waypoint.x && wp.y == closest_waypoint.y && wp.z == closest_waypoint.z;
    }) != circuit1_.end()) ? circuit1_ : circuit2_;

    auto it = std::find_if(selected_circuit.begin(), selected_circuit.end(), [&closest_waypoint](const atc_sim_ros2::msg::Waypoint& wp) {
        return wp.x == closest_waypoint.x && wp.y == closest_waypoint.y && wp.z == closest_waypoint.z;
    });

    if (it != selected_circuit.end()) {
         // Se obtiene el indice del wp mas cercano
        size_t closest_index = std::distance(selected_circuit.begin(), it);
        // El siguiente wp al mas cercano
        size_t next_index = (closest_index + 1) % selected_circuit.size();

        atc_sim_ros2::msg::Waypoint next_wp = selected_circuit[next_index];

        // Se genera la ruta completa del circuito de espera
        std::vector<atc_sim_ros2::msg::Waypoint> route = generateRoute(next_wp, avion_msg, selected_circuit);

        atc_sim_ros2::msg::WaypointUpdate update_msg;
        update_msg.avion_id = avion_msg.id;
        update_msg.waypoints = route;
        update_msg.speed = avion_msg.speed;
        waypoints_pub_->publish(update_msg);

        RCLCPP_INFO(this->get_logger(), "Se asignó el %s al avion %s", (&selected_circuit == &circuit1_ ? "circuito 1" : "circuito 2"), avion_msg.id.c_str());
        /*for (size_t i = 0; i<route.size();i++) {
            RCLCPP_INFO(this->get_logger(), "Waypoint %zu: [%.2f, %.2f, %.2f]", i, route[i].x, route[i].y, route[i].z);
        } */
    }
}

std::vector<atc_sim_ros2::msg::Waypoint> ATC::generateRoute(const atc_sim_ros2::msg::Waypoint start_waypoint, atc_sim_ros2::msg::Flight& avion_msg, const std::vector<atc_sim_ros2::msg::Waypoint>& circuit) {
    std::vector<atc_sim_ros2::msg::Waypoint> route;
    atc_sim_ros2::msg::Waypoint pos_avion;
    pos_avion.x = avion_msg.posx;
    pos_avion.y = avion_msg.posy;
    pos_avion.z = avion_msg.posz;

    // Primero encuentra la posicion del wp mas cercano
    auto it = std::find_if(circuit.begin(), circuit.end(), [&start_waypoint](const atc_sim_ros2::msg::Waypoint& wp) {
        return wp.x == start_waypoint.x && wp.y == start_waypoint.y && wp.z == start_waypoint.z;
    });

    if (it != circuit.end()) {
            
        std::vector<atc_sim_ros2::msg::Waypoint> intermedios = generateIntermediateWaypoints(pos_avion, start_waypoint, 1);
        route.insert(route.end(), intermedios.begin(), intermedios.end());
        // Primero se añaden los waypoints desde el más cercano al ultimo de la lista
        route.insert(route.end(), it, circuit.end());

        if (it != circuit.begin()) {
            // Y ahora desde el inicio de la lista hasta el más cercano
            route.insert(route.end(), circuit.begin(), it);
        }

        route.push_back(start_waypoint);
        
    }

    // Se generan los waypoints intermedios
    std::vector<atc_sim_ros2::msg::Waypoint> complete_route;
    for (size_t i = 0; i < route.size() - 1; i++) {
        const auto& start = route[i];
        const auto& end = route[i + 1];
        complete_route.push_back(start);

        std::vector<atc_sim_ros2::msg::Waypoint> intermedios = generateIntermediateWaypoints(start, end, 1);
        complete_route.insert(complete_route.end(), intermedios.begin(), intermedios.end());
    }
    complete_route.push_back(route.back());
    return complete_route;
}

atc_sim_ros2::msg::Waypoint ATC::findClosestWaypoint(double pos_x, double pos_y, double pos_z) {
    double min_distance = std::numeric_limits<double>::max();
    atc_sim_ros2::msg::Waypoint closest_waypoint;

    // Se busca en el primer circuito
    for (const auto& wp : circuit1_) {
        double dx = wp.x - pos_x;
        double dy = wp.y - pos_y;
        double dz = wp.z - pos_z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance < min_distance) {
            min_distance = distance;
            closest_waypoint = wp;
        }
    }
    // Y en el segundo
    for (const auto& wp : circuit2_) {
        double dx = wp.x - pos_x;
        double dy = wp.y - pos_y;
        double dz = wp.z - pos_z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance < min_distance) {
            min_distance = distance;
            closest_waypoint = wp;
        }
    }

    return closest_waypoint;
}

void ATC::adjustRoute(atc_sim_ros2::msg::Flight& avion) { 
    if (avion.waypoints.empty() ) {
        return;
    }

    const atc_sim_ros2::msg::Waypoint& first_wp = avion.waypoints.front();

    // Se busca el primer waypoint que pertenezca al circuito de espera
    auto it = std::find_if(circuit1_.begin(),circuit1_.end(), [&first_wp](const atc_sim_ros2::msg::Waypoint& wp) {
        return wp.x == first_wp.x && wp.y == first_wp.y && wp.z == first_wp.z;
    });

    if (it != circuit1_.end()) {
         // Se obtiene el indice del wp mas cercano
        size_t closest_index = std::distance(circuit1_.begin(), it);
        // El siguiente wp al mas cercano
        size_t next_index = (closest_index + 1) % circuit1_.size();

        atc_sim_ros2::msg::Waypoint next_wp = circuit1_[next_index];

        // Se eliminan los waypoints intermedios hasta el segundo waypoint del circuito
        auto wp_it = std::find_if(avion.waypoints.begin(), avion.waypoints.end(), [&next_wp](const atc_sim_ros2::msg::Waypoint& wp) {
            return wp.x == next_wp.x && wp.y == next_wp.y && wp.z == next_wp.z;
        });

        if (wp_it != avion.waypoints.end()) {
            avion.waypoints.erase(avion.waypoints.begin(), wp_it);
        }

        atc_sim_ros2::msg::Waypoint start;
        start.x = avion.posx;
        start.y = avion.posy;
        start.z = avion.posz;

        int num_intermedios = 2;
        std::vector<atc_sim_ros2::msg::Waypoint> intermedios = generateIntermediateWaypoints(start, next_wp, num_intermedios);
    
        // Se añaden los nuevos waypoints a los aviones
        avion.waypoints.insert(avion.waypoints.begin(), intermedios.begin(), intermedios.end());
    }

    atc_sim_ros2::msg::WaypointUpdate update_msg; 
    update_msg.avion_id = avion.id; 
    update_msg.waypoints = avion.waypoints; 
    update_msg.speed = avion.speed;
    waypoints_pub_->publish(update_msg); 
    //RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Ruta ajustada para el avion %s", avion.id.c_str()); 
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
           
           if (routesTooClose(aviones[i], aviones[j], 2.0)) { // Umbral de 2
                RCLCPP_WARN(this->get_logger(), " Rutas demasiado cercanas entre %s y %s", aviones[i].id.c_str(), aviones[j].id.c_str());
                adjustTrajectory(aviones[i], aviones[j]);
           }

           
           if (areRoutesIntersecting(aviones[i], aviones[j])) {
            RCLCPP_WARN(this->get_logger(), "Rutas en conflicto entre %s y %s", aviones[i].id.c_str(), aviones[j].id.c_str());
            adjustTrajectory(aviones[i], aviones[j]);
           }

           double distancia = calculateDistance(aviones[i], aviones[j]);
           if (distancia < 1) {
            RCLCPP_ERROR(this->get_logger(), "Colsion entre %s y %s", aviones[i].id.c_str(), aviones[j].id.c_str());
            contador_colisiones_++;

            // Publicar actualizacion para eliminar los aviones
            atc_sim_ros2::msg::WaypointUpdate update1, update2;
            update1.avion_id = aviones[i].id;
            update1.waypoints.clear(); // Lista de waypoints vacía para indicar eliminación

            update2.avion_id = aviones[j].id;
            update2.waypoints.clear();

            waypoints_pub_->publish(update1);
            waypoints_pub_->publish(update2);

            // Eliminar los aviones de la lista
            if (i < j) {
                aviones.erase(aviones.begin() + j);
                aviones.erase(aviones.begin() + i);
            } else {
                aviones.erase(aviones.begin() + i);
                aviones.erase(aviones.begin() + j);
            }
            break;
           }
        }
    }
    // Publica el total de colisiones detectadas
    RCLCPP_INFO(this->get_logger(), "Total colisiones detectadas: %d", contador_colisiones_/2);
}

bool ATC::routesTooClose(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2, double threshold_distance){
    if (avion1.waypoints.empty() || avion2.waypoints.empty()) {
        return false;
    }

    const int num_points = 50;
    std::vector<atc_sim_ros2::msg::Waypoint> puntos_trayectoria1 = generateTrajectoryPoints(avion1, avion1.waypoints[0], num_points);
    std::vector<atc_sim_ros2::msg::Waypoint> puntos_trayectoria2 = generateTrajectoryPoints(avion2, avion2.waypoints[0], num_points);

    //Compara cada punto de la trayectoria 1 con cada punto de la trayectoria 2
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& punto1 : puntos_trayectoria1) {
        for (const auto& punto2 : puntos_trayectoria2) {
            double dx = punto1.x - punto2.x;
            double dy = punto1.y - punto2.y;
            double dz = punto1.z - punto2.z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            min_distance = std::min(min_distance, distance);
        }
    }

    if (min_distance < threshold_distance) {
        //RCLCPP_WARN(this->get_logger(), "Proximidad detectada entre %s y %s con distancia %.2f", avion1.id.c_str(), avion2.id.c_str(), distance);
        return true;
    }
    return false;
}

std::vector<atc_sim_ros2::msg::Waypoint> ATC::generateTrajectoryPoints(const atc_sim_ros2::msg::Flight& avion, const atc_sim_ros2::msg::Waypoint& waypoint, int num_points) {
    std::vector<atc_sim_ros2::msg::Waypoint> trajectory_points;

    double step_x = (waypoint.x - avion.posx) / num_points;
    double step_y = (waypoint.y - avion.posy) / num_points;
    double step_z = (waypoint.z - avion.posz) / num_points;

    for (int i = 0; i <= num_points; i++) {
        atc_sim_ros2::msg::Waypoint point;
        point.x = avion.posx + step_x * i;
        point.y = avion.posy + step_y * i;
        point.z = avion.posz + step_z * i;
        trajectory_points.push_back(point);
    }

    return trajectory_points;
}

bool ATC::areTooClose(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2, double threshold_distance) {
    double dx = avion1.posx - avion2.posx;
    double dy = avion1.posy - avion2.posy;
    double dz = avion1.posz - avion2.posz;

    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    //RCLCPP_DEBUG(this->get_logger(), "Distancia actual entre %s y %s: %.2f", avion1.id.c_str(), avion2.id.c_str(), distance);

    return distance < threshold_distance;
}

std::optional<ATC::IntersectionResult> ATC::intersectLines3D(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2, double threshold_distance) {

    double bearingX1 = std::cos(avion1.bearing);
    double bearingY1 = std::sin(avion1.bearing);
    double bearingX2 = std::cos(avion2.bearing);
    double bearingY2 = std::sin(avion2.bearing);

    // Ecuaciones paramétricas
    // P1(t) = avion1.posx + bearingX1 * t; P1(t) = avion1.posy + bearingY1 * t
    // P1(t) = avion2.posx + bearingX2 * t; P1(t) = avion2.posy + bearingY2 * t

    // Se resuelve el sistema de ecuaciones para encontrar el punto de interseccion
    double denominator = (bearingX1 * bearingY2 - bearingY1 * bearingX2);

    if (std::abs(denominator) < 1e-6) {
        // Si el denominador es 0, las trayectorias son paralelas y no se intersectan
        // Se calcula la distancia entre las trayectorias para comprobar el umbral
        double dx1 = std::abs(avion2.posx - avion1.posx);
        double dy1 = std::abs(avion2.posy - avion1.posy);
        double dz1 = std::abs(avion2.posz - avion1.posz);
        double distance = std::sqrt(std::pow(bearingX1 * dy1 - bearingY1 * dx1, 2) + std::pow(dz1, 2));
        if (distance < threshold_distance) {
            return IntersectionResult{0.0, 0.0, 0.0, 0.0, 0.0}; // No hay interseccion exacta
        }
        return IntersectionResult{0.0, 0.0, 0.0, 0.0, 0.0};
    }

    double t1 = ((avion2.posx - avion1.posx) * bearingY2 - (avion2.posy - avion1.posy) * bearingX2) / denominator;

    // Coordenadas de interseccion
    double x_intersection = avion1.posx + bearingX1 * t1;
    double y_intersection = avion1.posy + bearingY1 * t1; 
    double z_intersection = avion1.posz + (avion2.posz - avion1.posz) * t1;

    // Se verifica la interseccion dentro del umbral
    double distance = std::sqrt(std::pow(x_intersection - avion1.posx, 2) +
                                std::pow(y_intersection - avion1.posy, 2) +
                                std::pow(z_intersection - avion1.posz, 2));
    if (distance < threshold_distance) {
        return IntersectionResult{x_intersection, y_intersection, z_intersection, t1, 0.0}; // t2 no se usa en la función, se deja como 0.0
    }

    return std::nullopt;
}

bool ATC::areRoutesIntersecting(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2) {
     if (avion1.waypoints.empty() || avion2.waypoints.empty()) {
        return false;
    }
    double threshold_distance = 1.0;
    // Se verifica si ambos aviones tienen el mismo waypoint o similar
    if (std::abs(avion1.waypoints[0].x - avion2.waypoints[0].x) <= threshold_distance &&
        std::abs(avion1.waypoints[0].y - avion2.waypoints[0].y) <= threshold_distance &&
        std::abs(avion1.waypoints[0].z - avion2.waypoints[0].z) <= threshold_distance) {
            double time_to_wp1 = calculateTimeToWaypoint(avion1, avion1.waypoints[0].x, avion1.waypoints[0].y, avion1.waypoints[0].z);
            double time_to_wp2 = calculateTimeToWaypoint(avion2, avion2.waypoints[0].x, avion2.waypoints[0].y, avion2.waypoints[0].z);
            

            if (std::abs(time_to_wp1 - time_to_wp2) < 0.2) { // No son 0.06 segundos si no 6 "simulaciones"
               //RCLCPP_WARN(this->get_logger(), "Intersección detectada POR MISMO WAYPOINT entre %s y %s ", avion1.id.c_str(), avion2.id.c_str());
               //RCLCPP_INFO(this->get_logger(), "Tiempo: %.2f, %.2f . Dif: %.2f", time_to_wp1, time_to_wp2, std::abs(time_to_wp1 - time_to_wp2));
                return true;
            }
    }

    auto result = intersectLines3D(avion1, avion2, threshold_distance);
    if (!result) {
        return false; // No hay interseccion en las trayectorias
    }

    double time_to_wp1 = calculateTimeToWaypoint(avion1, result->x, result->y, result->z);
    double time_to_wp2 = calculateTimeToWaypoint(avion2, result->x, result->y, result->z);

    if (std::abs(time_to_wp1 - time_to_wp2) < 0.2) {
        //RCLCPP_WARN(this->get_logger(), "Intersección detectada entre %s y %s en (%.2f, %.2f, %.2f)", avion1.id.c_str(), avion2.id.c_str(), result->x, result->y, result->z);
        //RCLCPP_INFO(this->get_logger(), "Tiempo: %.2f, %.2f . Dif: %.2f", time_to_wp1, time_to_wp2, std::abs(time_to_wp1 - time_to_wp2));
        return true;
    }

    return false;
}

double ATC::calculateTimeToWaypoint(const atc_sim_ros2::msg::Flight& avion, double x, double y, double z) {
    double dx = x - avion.posx;
    double dy = y - avion.posy;
    double dz = z - avion.posz;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    double speed = avion.speed;
    double time = distance / speed;
    return time;
}

bool ATC::checkWaypointInCircuitOrClose(const atc_sim_ros2::msg::Flight& avion) {
    if (avion.waypoints.empty()) {
        return false;
    }

    const auto& next_wp = avion.waypoints.front();
    atc_sim_ros2::msg::Waypoint start;
    start.x = avion.posx;
    start.y = avion.posy;
    start.z = avion.posz;

    double threshold_distance = 3.0; // Distancia al wp del circuito

    for (const auto& circuit_wp : circuit1_) {
        if ((next_wp.x == circuit_wp.x && next_wp.y == circuit_wp.y && next_wp.z == circuit_wp.z) || distanceBetweenWaypoints(start, circuit_wp) <= threshold_distance) {
            return true;
        }
    }

    for (const auto& circuit_wp : circuit2_) {
        if ((next_wp.x == circuit_wp.x && next_wp.y == circuit_wp.y && next_wp.z == circuit_wp.z) || distanceBetweenWaypoints(start, circuit_wp) <= threshold_distance) {
            return true;
        }
    }

    return false;
}

bool ATC::checkWaypointExit(const atc_sim_ros2::msg::Flight& avion) {
    if (avion.waypoints.empty()) {
        return false;
    }

    const auto& next_wp = avion.waypoints.front();
    // Distancia al wp de salida del circuito

    if ((next_wp.x == 20.0 && next_wp.y == 30.0 && next_wp.z == 5.0) ||    // Salida circuito 1
        (next_wp.x == -20.0 && next_wp.y == 30.0 && next_wp.z == 5.0) ) {  // Salida circuito 2
        return true;
    }
    return false;
}

void ATC::adjustTrajectory(atc_sim_ros2::msg::Flight& avion1, atc_sim_ros2::msg::Flight& avion2) {
    bool avoidCollision = false;
    RCLCPP_INFO(this->get_logger(), "Ajustando la trayectoria para %s y %s", avion1.id.c_str(), avion2.id.c_str());

    // Se comprueba si el siguiente waypoint de los aviones está en el circuito o cerca de uno
    bool avion1_near_circuit = checkWaypointInCircuitOrClose(avion1);
    bool avion2_near_circuit = checkWaypointInCircuitOrClose(avion2);

    bool avion1_exit = checkWaypointExit(avion1);
    bool avion2_exit = checkWaypointExit(avion2);

    // Paso 1: Ajustar la velocidad de los aviones
    adjustSpeed(avion1, avion2);
    if (checkCollisionAfterAdjustment(avion1, avion2)) { // Se verifica si la colisión sigue ocurriendo
        avoidCollision = true;
    }
    
    // Si no se ha evitado la colision con la velocidad se intenta ajustando la altura
    if (!avoidCollision) {
        if (avion1_near_circuit) {
            adjustRoute(avion1);
        } else if (avion2_near_circuit){
            adjustRoute(avion2);
        } else {
            // Paso 2: Ajustar la altitud de ambos aviones
            adjustAltitude(avion1, avion2);
        }
        
        if (checkCollisionAfterAdjustment(avion1, avion2)) { // Se verifica si la colisión sigue ocurriendo
            avoidCollision = true;
        }
    }
    
    // Si aún no se evitó la colisión se intenta ajustando los waypoints lateralmente
    if (!avoidCollision) {
        if (avion1_near_circuit) {
            adjustRoute(avion1);
        } else if (avion2_near_circuit) {
            adjustRoute(avion2);
        } else if (!avion1_exit && !avion2_exit){ // No ajustar los waypoints si alguno de los aviones está en ruta de salida
            // Paso 3: Desplazar los waypoints de uno de los aviones
            // Este paso NO se puede aplicar a un avion que esté en el punto de salida
            adjustNextWaypoint(avion1, avion2);
        }
        
        if (checkCollisionAfterAdjustment(avion1, avion2)) { // Se verifica si la colisión sigue ocurriendo
            avoidCollision = true;
        }
    }

    if (!avoidCollision) {
        adjustSpeed(avion1, avion2);
        if (checkCollisionAfterAdjustment(avion1, avion2)) { // Se verifica si la colisión sigue ocurriendo
            avoidCollision = true;
        }
    }
    
    
    
}

bool ATC::checkCollisionAfterAdjustment(atc_sim_ros2::msg::Flight& avion1, atc_sim_ros2::msg::Flight& avion2) {
    if (routesTooClose(avion1, avion2, 1.0)) { // Umbral de 1
        RCLCPP_INFO(this->get_logger(), "Colisión no evitada después del ajuste");
        return false; // No se evitó la colisión
    }
    if (areRoutesIntersecting(avion1, avion2)){
        RCLCPP_INFO(this->get_logger(), "Colisión no evitada después del ajuste");
        return false; // No se evitó la colisión
    }
    return true; // Se evitó la colisión
}

void ATC::adjustAltitude(atc_sim_ros2::msg::Flight& avion1, atc_sim_ros2::msg::Flight& avion2) {
    if (avion1.waypoints.empty() || avion2.waypoints.empty()) {
        return;
    }

    // Se eliminan todos los waypoints intermedios de ambos aviones excepto el destino
    if (avion1.waypoints.size() <=1 || avion2.waypoints.size() <= 1) {
        return;
    }

    // Se busca el primer waypoint que pertenezca al circuito de espera
    auto it1 = std::find_if(avion1.waypoints.begin(), avion1.waypoints.end(), [&avion1](const atc_sim_ros2::msg::Waypoint& wp) {
        const auto& start_waypoint = *avion1.waypoints.begin();
        return wp.x == start_waypoint.x && wp.y == start_waypoint.y && wp.z == start_waypoint.z;
    });

    auto it2 = std::find_if(avion2.waypoints.begin(), avion2.waypoints.end(), [&avion2](const atc_sim_ros2::msg::Waypoint& wp) {
        const auto& start_waypoint2 = *avion2.waypoints.begin();
        return wp.x == start_waypoint2.x && wp.y == start_waypoint2.y && wp.z == start_waypoint2.z;
    });

    if (it1 != avion1.waypoints.end()) {
        // Se eliminan los waypoints intermedios hasta el primer waypoint del circuito
        avion1.waypoints.erase(avion1.waypoints.begin(), it1);
    }

    if (it2 != avion2.waypoints.end()) {
        // Se eliminan los waypoints intermedios hasta el primer waypoint del circuito
        avion2.waypoints.erase(avion2.waypoints.begin(), it2); 
    }
    // El primer waypoint del circuito se considera el destino final
    auto final_destination1 = avion1.waypoints.front();
    auto final_destination2 = avion2.waypoints.front();

    // Se decide que avion sube en funcion de la posicion en z
    bool avion1_sube = avion1.posz > avion2.posz;

    // Parámetros max y min
    double delta_altitude = 1.0;
    double max_altitude = 10.0;
    double min_altitude = 0.0;

    // Se calcula el desplazamiento para el primer waypoint segun el bearing
    double avance_distancia = 1.5;
    double bearing_rad1 = avion1.bearing * M_PI / 180.0;
    double bearing_rad2 = avion2.bearing * M_PI / 180.0;

    // Se crea un nuevo waypoint inicial con ajuste en Z
    atc_sim_ros2::msg::Waypoint nuevo_wp1, nuevo_wp2;

    nuevo_wp1.x = avion1.posx + avance_distancia * cos(bearing_rad1); 
    nuevo_wp1.y = avion1.posy + avance_distancia * sin(bearing_rad1);

    nuevo_wp2.x = avion2.posx + avance_distancia * cos(bearing_rad2);
    nuevo_wp2.y = avion2.posy + avance_distancia * sin(bearing_rad2);

    if (avion1.posz < avion2.posz){
        // Avion2 sube
        nuevo_wp1.z = std::max(avion1.posz - delta_altitude, min_altitude);
        nuevo_wp2.z = std::min(avion2.posz + delta_altitude, max_altitude);
    } else {
        // Avion1 sube
        nuevo_wp1.z = std::min(avion1.posz + delta_altitude, max_altitude);
        nuevo_wp2.z = std::max(avion2.posz - delta_altitude, min_altitude);
    }

     // Se generan nuevos waypoints intermedios
    int num_intermedios = 1;
    std::vector<atc_sim_ros2::msg::Waypoint> waypoints1 = generateIntermediateWaypoints(nuevo_wp1, final_destination1, num_intermedios);
    std::vector<atc_sim_ros2::msg::Waypoint> waypoints2 = generateIntermediateWaypoints(nuevo_wp2, final_destination2, num_intermedios);

    // Se añaden los nuevos waypoints a los aviones
    avion1.waypoints.insert(avion1.waypoints.begin(), waypoints1.begin(), waypoints1.end());
    avion2.waypoints.insert(avion2.waypoints.begin(), waypoints2.begin(), waypoints2.end());

    // Se publican los nuevos waypoints para ambos aviones
    atc_sim_ros2::msg::WaypointUpdate update_msg1;
    update_msg1.avion_id = avion1.id;
    update_msg1.waypoints = avion1.waypoints;
    update_msg1.speed = avion1.speed;
    waypoints_pub_->publish(update_msg1);

    atc_sim_ros2::msg::WaypointUpdate update_msg2;
    update_msg2.avion_id = avion2.id;
    update_msg2.waypoints = avion2.waypoints;
    update_msg2.speed = avion2.speed;
    waypoints_pub_->publish(update_msg2);

    RCLCPP_INFO(this->get_logger(), "Altitud ajustada: El avion %s %s y el avion %s %s", avion1.id.c_str(), avion1_sube ? "sube" : "baja", avion2.id.c_str(), avion1_sube ? "baja" : "sube");
   

/*
    auto final_destination1 = avion1.waypoints.back();
    avion1.waypoints.clear();
    avion1.waypoints.push_back(final_destination1);

    auto final_destination2 = avion2.waypoints.back();
        avion2.waypoints.clear();
        avion2.waypoints.push_back(final_destination2);
    // Se decide que avion sube en funcion de la posicion en z
    bool avion1_sube = avion1.posz > avion2.posz;

    // Parámetros max y min
    double delta_altitude = 1.0;
    double max_altitude = 10.0;
    double min_altitude = 0.0;

    // Se calcula el desplazamiento para el primer waypoint segun el bearing
    double avance_distancia = 1.5;
    double bearing_rad1 = avion1.bearing * M_PI / 180.0;
    double bearing_rad2 = avion2.bearing * M_PI / 180.0;

    // Se crea un nuevo waypoint inicial con ajuste en Z
    atc_sim_ros2::msg::Waypoint nuevo_wp1, nuevo_wp2;

    nuevo_wp1.x = avion1.posx + avance_distancia * cos(bearing_rad1); 
    nuevo_wp1.y = avion1.posy + avance_distancia * sin(bearing_rad1);

    nuevo_wp2.x = avion2.posx + avance_distancia * cos(bearing_rad2);
    nuevo_wp2.y = avion2.posy + avance_distancia * sin(bearing_rad2);

    if (avion1.posz < avion2.posz){
        // Avion2 sube
        nuevo_wp1.z = std::max(avion1.posz - delta_altitude, min_altitude);
        nuevo_wp2.z = std::min(avion2.posz + delta_altitude, max_altitude);
    } else {
        // Avion1 sube
        nuevo_wp1.z = std::min(avion1.posz + delta_altitude, max_altitude);
        nuevo_wp2.z = std::max(avion2.posz - delta_altitude, min_altitude);
    }

    // Se generan nuevos waypoints intermedios
    int num_intermedios = 2;
    std::vector<atc_sim_ros2::msg::Waypoint> waypoints1 = generateIntermediateWaypoints(nuevo_wp1, final_destination1, num_intermedios);
    std::vector<atc_sim_ros2::msg::Waypoint> waypoints2 = generateIntermediateWaypoints(nuevo_wp2, final_destination2, num_intermedios);

    // Se añaden los nuevos waypoints a los aviones
    avion1.waypoints.insert(avion1.waypoints.begin(), waypoints1.begin(), waypoints1.end());
    avion2.waypoints.insert(avion2.waypoints.begin(), waypoints2.begin(), waypoints2.end());

    // Se publican los nuevos waypoints para ambos aviones
    atc_sim_ros2::msg::WaypointUpdate update_msg1;
    update_msg1.avion_id = avion1.id;
    update_msg1.waypoints = avion1.waypoints;
    update_msg1.speed = avion1.speed;
    waypoints_pub_->publish(update_msg1);

    atc_sim_ros2::msg::WaypointUpdate update_msg2;
    update_msg2.avion_id = avion2.id;
    update_msg2.waypoints = avion2.waypoints;
    update_msg2.speed = avion2.speed;
    waypoints_pub_->publish(update_msg2);

    RCLCPP_INFO(this->get_logger(), "Altitud ajustada: El avion %s %s y el avion %s %s", avion1.id.c_str(), avion1_sube ? "sube" : "baja", avion2.id.c_str(), avion1_sube ? "baja" : "sube");
   */
}

void ATC::adjustNextWaypoint(atc_sim_ros2::msg::Flight& avion1, atc_sim_ros2::msg::Flight& avion2) {
    if (avion1.waypoints.empty() || avion2.waypoints.empty()) {
        return;
    }

    // Se eliminan todos los waypoints intermedios de ambos aviones excepto el destino
    if (avion1.waypoints.size() <=1 || avion2.waypoints.size() <= 1) {
        return;
    }
// Se busca el primer waypoint que pertenezca al circuito de espera
    auto it1 = std::find_if(avion1.waypoints.begin(), avion1.waypoints.end(), [&avion1](const atc_sim_ros2::msg::Waypoint& wp) {
        const auto& start_waypoint = *avion1.waypoints.begin();
        return wp.x == start_waypoint.x && wp.y == start_waypoint.y && wp.z == start_waypoint.z;
    });

    auto it2 = std::find_if(avion2.waypoints.begin(), avion2.waypoints.end(), [&avion2](const atc_sim_ros2::msg::Waypoint& wp) {
        const auto& start_waypoint2 = *avion2.waypoints.begin();
        return wp.x == start_waypoint2.x && wp.y == start_waypoint2.y && wp.z == start_waypoint2.z;
    });
    
     if (it1 != avion1.waypoints.end()) {
        // Se eliminan los waypoints intermedios hasta el primer waypoint del circuito
        avion1.waypoints.erase(avion1.waypoints.begin(), it1);
    }

    if (it2 != avion2.waypoints.end()) {
        // Se eliminan los waypoints intermedios hasta el primer waypoint del circuito
        avion2.waypoints.erase(avion2.waypoints.begin(), it2); 
    }

    // El primer waypoint del circuito se considera el destino final
    auto final_destination1 = avion1.waypoints.front();
    auto final_destination2 = avion2.waypoints.front();

    // Se calcula la direccion relativa entre los dos aviones
    double dx = final_destination2.x - final_destination1.x;
    double dy = final_destination2.y - final_destination1.y;

    atc_sim_ros2::msg::Waypoint new_wp1;

    if (dx * dy > 0) { //Si ambos aviones se mueven en la misma direccion 
        
        // Se genera un nuevo wp para el avion 1 desplazado a la derecha
        new_wp1 = createLateralDisplacedWaypoint(avion1, true);
        
    } else {
        // Se desplaza a la izquierda
        new_wp1 = createLateralDisplacedWaypoint(avion1, false);
    }
    
    // Se añade el waypoint desplazado al inicio de la lista de waypoints de cada avion
    avion1.waypoints.insert(avion1.waypoints.begin(), new_wp1);
    // Se generan los nuevos waypoints intermedios
    std::vector<atc_sim_ros2::msg::Waypoint> new_waypoints1 = generateIntermediateWaypoints(new_wp1, final_destination1, 1);
    // Se insertan los waypoints intermedios
    avion1.waypoints.insert(avion1.waypoints.begin() + 1, new_waypoints1.begin(), new_waypoints1.end());
    // Publicar la actualización
    atc_sim_ros2::msg::WaypointUpdate update_msg1;
    update_msg1.avion_id = avion1.id;
    update_msg1.waypoints = avion1.waypoints;
    update_msg1.speed = avion1.speed;
    waypoints_pub_->publish(update_msg1);
    RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Ajustando waypoint del avion %s ", avion1.id.c_str());
    /*
    auto final_destination1 = avion1.waypoints.back();
    auto final_destination2 = avion2.waypoints.back();

    // Se calcula la direccion relativa entre los dos aviones
    double dx = final_destination2.x - final_destination1.x;
    double dy = final_destination2.y - final_destination1.y;

    avion1.waypoints.clear();
    avion1.waypoints.push_back(final_destination1);
    atc_sim_ros2::msg::Waypoint new_wp1;

    if (dx * dy > 0) { //Si ambos aviones se mueven en la misma direccion 
        
        // Se genera un nuevo wp para el avion 1 desplazado a la derecha
        new_wp1 = createLateralDisplacedWaypoint(avion1, true);
        
    } else {
        // Se desplaza a la izquierda
        new_wp1 = createLateralDisplacedWaypoint(avion1, false);
    }
    
    // Se añade el waypoint desplazado al inicio de la lista de waypoints de cada avion
    avion1.waypoints.insert(avion1.waypoints.begin(), new_wp1);
    // Se generan los nuevos waypoints intermedios
    std::vector<atc_sim_ros2::msg::Waypoint> new_waypoints1 = generateIntermediateWaypoints(new_wp1, final_destination1, 2);
    // Se insertan los waypoints intermedios
    avion1.waypoints.insert(avion1.waypoints.begin() + 1, new_waypoints1.begin(), new_waypoints1.end());
    // Publicar la actualización
    atc_sim_ros2::msg::WaypointUpdate update_msg1;
    update_msg1.avion_id = avion1.id;
    update_msg1.waypoints = avion1.waypoints;
    update_msg1.speed = avion1.speed;
    waypoints_pub_->publish(update_msg1);
    RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Ajustando waypoint del avion %s ", avion1.id.c_str());
    */
}   

atc_sim_ros2::msg::Waypoint ATC::createLateralDisplacedWaypoint(atc_sim_ros2::msg::Flight& avion, bool moveRight) {
    atc_sim_ros2::msg::Waypoint wp;
    double displacement = moveRight ? 1.0 : -1.0; // Desplazamiento lateral de una unidad
    double perpendicular_angle = avion.bearing + M_PI_2;

    // Cálculo de la nueva posición desplazada lateralmente
    wp.x = avion.posx + displacement * std::cos(perpendicular_angle);
    wp.y = avion.posy + displacement * std::sin(perpendicular_angle);
    wp.z = avion.posz;
    return wp;
}

void ATC::adjustSpeed(atc_sim_ros2::msg::Flight& avion1, atc_sim_ros2::msg::Flight& avion2) {
    //RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Ajustando velocidades de %s y %s para evitar colision", avion1.id.c_str(), avion2.id.c_str());
    // Parámetros configurables
    const double min_speed = 10.0;
    const double max_speed = 30.0;
    const double speed_adjustment = 5.0;
    double threshold_distance = 1.0;

    // Primero se calcula el punto de interseccion
    auto result = intersectLines3D(avion1, avion2, threshold_distance);

    // Se calcula la distancia del avion 1 al punto de interseccion
    double dx1 = result->x - avion1.posx;
    double dy1 = result->y - avion1.posy;
    double dz1 = result->z - avion1.posz;
    double distance1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + dz1 * dz1);

    // Se calcula la distancia del avion 2 al punto de interseccion
    double dx2 = result->x - avion2.posx;
    double dy2 = result->y - avion2.posy;
    double dz2 = result->z - avion2.posz;
    double distance2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);

    if (distance1 < distance2){
        //Avion 1 está mas cerca, aumenta velocidad
        avion1.speed = std::min(avion1.speed + speed_adjustment, max_speed);
        avion2.speed = std::max(avion2.speed - speed_adjustment, min_speed);
    } else {
        // Avion 2 está más cerca, reduce su velocidad
        //Avion 1 está mas cerca, aumenta velocidad
        avion2.speed = std::min(avion2.speed + speed_adjustment, max_speed);
        avion1.speed = std::max(avion1.speed - speed_adjustment, min_speed);
    }

    // Se publican las velocidades actualizadas
    atc_sim_ros2::msg::WaypointUpdate update_msg1;
    update_msg1.avion_id = avion1.id;
    update_msg1.waypoints = avion1.waypoints;
    update_msg1.speed = avion1.speed;

    atc_sim_ros2::msg::WaypointUpdate update_msg2;
    update_msg2.avion_id = avion2.id;
    update_msg2.waypoints = avion2.waypoints;
    update_msg2.speed = avion2.speed;

    waypoints_pub_->publish(update_msg1);
    waypoints_pub_->publish(update_msg2);

    RCLCPP_INFO(rclcpp::get_logger("atc_logger"), "Velocidades ajustadas: %s (%.2f) y %s (%.2f)", avion1.id.c_str(), avion1.speed, avion2.id.c_str(), avion2.speed);
    /*
    // Calcular diferencias angulares
    double angle_diff1 = normalizeAngle(calculateBearing(avion1, avion2) - avion1.bearing);
    double angle_diff2 = normalizeAngle(calculateBearing(avion2, avion1) - avion2.bearing);

    // Diferencia de rumbos, para saber si avanzan en direcciones similares
    double bearing_diff = std::abs(normalizeAngle(avion1.bearing - avion2. bearing));

    // Configuracion valida, uno detrás del otro:
    // 1. Uno de los aviones debe estar en el cono frontal del otro
    // 2. Sus rumbos deben ser parecidos, +- 30 grados
    bool one_behind_other = 
        (std::abs(angle_diff1) < M_PI_2) && // Avion 1 en cono frontal de Avion 2
        (std::abs(angle_diff2) > M_PI_2) && // Avion 2 fuera del cono frontal de Avion 1
        (std::abs(bearing_diff) < M_PI / 6); // Rumbos compatibles
    
    // Configuracion valida, trayectorias cruzadas
    bool crossing_trajectories = (bearing_diff >= M_PI / 6 && bearing_diff <= 5 * M_PI / 9); // Diferencia de bearing entre 30º y 100º
    
    if (!one_behind_other && !crossing_trajectories) {
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
    } else if (crossing_trajectories) { // El que esté más cerca del punto de cruce acelera
        if (angle_diff1 < 0) {
            avion1.speed = std::min(avion1.speed + speed_adjustment, max_speed);
            avion2.speed = std::max(avion2.speed - speed_adjustment, min_speed);
        } else {
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
    */
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

double ATC::distanceBetweenWaypoints( const atc_sim_ros2::msg::Waypoint& wp1, const atc_sim_ros2::msg::Waypoint& wp2) {
    double dx = wp1.x - wp2.x;
    double dy = wp1.y - wp2.y;
    double dz = wp1.z - wp2.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    return distance;
}