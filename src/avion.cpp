#include "atc_sim_ros2/avion.hpp"
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/waypoint.hpp"


Avion::Avion() 
{
    
    airline_ = assignRandomAirline();
    id_ = generateRandomID();
    posx_ = generateRandomPosition(true);
    posy_ = generateRandomPosition(true);
    posz_ = generateRandomPosition(false);
    speed_ = 10.0;
    bearing_ =generateRandomBearing();
    reached_waypoint_ = false;
}

std::string Avion::getID() const { return id_; }
std::string Avion::getAirline() const {return airline_; }
double Avion::getPosX() const { return posx_; }
double Avion::getPosY() const { return posy_; }
double Avion::getPosZ() const { return posz_; }
double Avion::getSpeed() const { return speed_; }
double Avion::getBearing() const { return bearing_; }

void Avion::addWaypoints(const std::vector<atc_sim_ros2::msg::Waypoint>& waypoints) {
    for (const auto& wp : waypoints) {
        waypoints_.push_back(wp);
    }
    RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Waypoints agregados: %zu ", waypoints_.size());
}

const std::vector<atc_sim_ros2::msg::Waypoint>& Avion::getWaypoints() const {
        return waypoints_;
}

std::string Avion::assignRandomAirline() {
    std::vector<std::string> airlines = {
        "Ryanair", "Iberia", "Air France", "British Airways", "Vueling"
    };

    int random_index = rand() % airlines.size();
    return airlines[random_index];
}

//Funcion que genera de manera aleatoria el ID
std::string Avion::generateRandomID()
{
    std::string prefix;
    // Asigna prefijo segun aerolinea
    if (airline_ == "Iberia") {
        prefix = "IB";
    } else if (airline_ == "Ryanair") {
        prefix = "RY";
    } else if (airline_ == "British Airways") {
        prefix = "BA";
    } else if (airline_ == "Air France") {
        prefix = "AF";
    } else if (airline_ == "Vueling") {
        prefix = "VL";
    }

    int random_id = rand() % 900 + 100;
    
    return prefix + std::to_string(random_id);
}

//Funcion para que los aviones se generen en los margenes que se establezcan
double Avion::generateRandomPosition(bool margins)
{
    double random_value = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX));

    if (margins) {
        if (random_value < 0.5) {
        // Se genera en margen negativo
        return -10.0 + (random_value * 1.0);
    } else {
        // Se genera en margen positivo
        return 9.0 + ((random_value - 0.5) * 1.0);
    }
    } else {
        return -5.0 + random_value * 10.0;
    }
    
}

//Funcion que genera de manera aleatoria un valor entre 0 y 2pi
double Avion::generateRandomBearing()
{
    return static_cast<double>(rand()) / (static_cast<double>(RAND_MAX)) * 2 * M_PI;
} 

// Funcion para seleccionar waypoint aleatorio
void Avion::selectRandomWaypoint()
{
    // Se verifica si hay waypoints disponibles
    if (waypoints_.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("avion_logger"), "No hay waypoints disponibles");
        return;
    }
    int selected_waypoint = rand() % waypoints_.size();
    RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Índice seleccionado: %d, Tamaño de waypoints: %zu", selected_waypoint, waypoints_.size());
    target_waypoint_ = waypoints_[selected_waypoint];

    RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Avion %s dirigiendose al waypoint %d", id_.c_str(), selected_waypoint);
}

// Metodo para actualizar la posicion
void Avion::update(double delta_time)
{
    // Primero se calcula el angulo hacia el waypoint
    double dx = target_waypoint_.x - posx_;
    double dy = target_waypoint_.y - posy_;
    double dz = target_waypoint_.z - posz_;
    double distance_to_waypoint = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (!reached_waypoint_){
        // Margen de distancia al waypoint
        double waypoint_threshold = 0.2;

        if (distance_to_waypoint < waypoint_threshold) {
            reached_waypoint_ = true;
        } else {
            // Bearing objetivo
            double target_bearing = std::atan2(dy, dx);
            
            // Se ajusta el angulo de manero gradual
            double angle_difference = target_bearing - bearing_;
            if (angle_difference > M_PI) angle_difference -= 2 * M_PI;
            if (angle_difference < -M_PI) angle_difference += 2 * M_PI;

            double rotation_speed = 5.0 * delta_time;
            if (std::abs(angle_difference) < rotation_speed) {
            bearing_ = target_bearing;
            } else {
            bearing_ += (angle_difference > 0 ? rotation_speed : -rotation_speed);
            }

            double climb_rate = 1.0 * delta_time;
            if (std::abs(dz) < climb_rate) {
                posz_ = target_waypoint_.z;
            } else {
                posz_ += (dz > 0 ? climb_rate : -climb_rate);
            }


        }

    }

    double distance = speed_ * delta_time;

    posx_ += distance * std::cos(bearing_);
    posy_ += distance * std::sin(bearing_);
}

