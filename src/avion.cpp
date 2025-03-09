#include "atc_sim_ros2/avion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/waypoint.hpp"
#include <array>
#include <cmath>


Avion::Avion() 
{
    airline_ = assignRandomAirline();
    id_ = generateRandomID();
    posx_ = generateRandomPosition(true, false);
    posy_ = generateRandomPosition(true, false);
    posz_ = generateRandomPosition(false, true);
    bearing_ = generateBearing();
    speed_ = 20.0;
    reached_waypoint_ = false;
    ruta_completada_ = false;
    landings_ = 0;
}

std::string Avion::getID() const { return id_; }
std::string Avion::getAirline() const {return airline_; }
double Avion::getPosX() const { return posx_; }
double Avion::getPosY() const { return posy_; }
double Avion::getPosZ() const { return posz_; }
double Avion::getSpeed() const { return speed_; }
double Avion::getBearing() const { return bearing_; }

void Avion::addWaypoints(const std::vector<atc_sim_ros2::msg::Waypoint>& waypoints) {
    if (waypoints.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("avion_logger"), "No se proporcionaron waypoints");
        return;
    }
    for (const auto& wp : waypoints) {
        waypoints_.push_back(wp);
    }
    //RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Waypoints agregados: %zu ", waypoints_.size());
}

void Avion::clearWaypoints() {
    waypoints_.clear();
    //RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Waypoints eliminados ");
}

void Avion::setSpeed(double speed) {
    this->speed_ = speed;
}

void Avion::setPosX(double x) {
    this->posx_ = x;
}

void Avion::setPosY(double y) {
    this->posy_ = y;
}

void Avion::setPosZ(double z) {
    this->posz_ = z;
}

void Avion::setBearing(double bearing) {
    this->bearing_ = bearing;
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
double Avion::generateRandomPosition(bool margins, bool isAltitud)
{
    double random_value = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX));

    if(isAltitud) {
        // En el caso de altitud que siempre se genere un valor positivo
        return 1.0 + random_value * 5.0;  // Altitud entre 1 y 6
    } else if (margins) {
        // Generar posicion en los margenes de un cuadrado 20x20
        double side_position = 20.0;
        double random_pos = -20.0 + random_value * 40.0;

        int border_choice = rand() % 4;

        switch (border_choice) {
            case 0:
                return (rand() % 2 ==0) ? -side_position : side_position;
            case 1:
                return random_pos;
            case 2:
                return (rand() % 2 == 0) ? -side_position : side_position;
            case 3:
                return random_pos;
        }
    }
    return -20.0 + random_value * 40.0;
    
}

//Funcion que calcula el angulo hacia el origen (0,0) 
double Avion::generateBearing()
{
    double dx = -posx_;
    double dy = -posy_;

    return std::atan2(dy, dx);
} 

// Funcion para seleccionar waypoint aleatorio
void Avion::selectRandomWaypoint()
{
    // Se verifica si hay waypoints disponibles
    if (waypoints_.empty()) {
        //RCLCPP_WARN(rclcpp::get_logger("avion_logger"), "No hay waypoints disponibles");
        return;
    }
    int selected_waypoint = rand() % waypoints_.size();
    //RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Índice seleccionado: %d, Tamaño de waypoints: %zu", selected_waypoint, waypoints_.size());
    target_waypoint_ = waypoints_[selected_waypoint];

    //RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Avion %s dirigiendose al waypoint %d", id_.c_str(), selected_waypoint);
}

// Funcion para generar waypoints intermedios
std::vector<atc_sim_ros2::msg::Waypoint> Avion::generateIntermediateWaypoints(const atc_sim_ros2::msg::Waypoint& start, const atc_sim_ros2::msg::Waypoint& end, int num_points)
{
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

// Metodo para actualizar la posicion
void Avion::update(double delta_time)
{
    if (!waypoints_.empty()) {
        target_waypoint_ = waypoints_.front();
        //RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Avion %s dirigiendose al waypoint en posicion {%.2f, %.2f, %.2f}", id_.c_str(), target_waypoint_.x, target_waypoint_.y, target_waypoint_.z);
    }
    
   // Primero se calcula el angulo hacia el waypoint
    double dx = target_waypoint_.x - posx_;
    double dy = target_waypoint_.y - posy_;
    double dz = target_waypoint_.z - posz_;
    double distance_to_waypoint = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (!reached_waypoint_){
        // Margen de distancia al waypoint
        double waypoint_threshold = 2;
        //RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Distance to waypoint: %.2f", distance_to_waypoint);

        if (distance_to_waypoint < waypoint_threshold) {
            reached_waypoint_ = true;
            waypoints_.erase(waypoints_.begin());
            //RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Waypoint reached. Remaining waypoints:%zu", waypoints_.size());

            if (!waypoints_.empty()) {
                reached_waypoint_ = false;
            }

            if (waypoints_.empty()) {
                ruta_completada_ = true;
                landings_ += 1;
                RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Aterrizajes completados: %d", landings_);
                //RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Avion ID: %s | Ruta completada: %s | Waypoints vacios: %s", id_.c_str(), ruta_completada_ ? "Sí" : "No", waypoints_.empty() ? "Sí" : "No");
        
            }
            
        } else {
            // Bearing objetivo
            double target_bearing = std::atan2(dy, dx);
            // Se ajusta el angulo de manera gradual
            double angle_difference = target_bearing - bearing_;
            if (angle_difference > M_PI) angle_difference -= 2 * M_PI;
            if (angle_difference < -M_PI) angle_difference += 2 * M_PI;

            double rotation_speed = 10.0 * delta_time;
            if (std::abs(angle_difference) < rotation_speed) {
            bearing_ = target_bearing;
            } else {
            bearing_ += (angle_difference > 0 ? rotation_speed : -rotation_speed);
            }
           //bearing_ += (std::abs(angle_difference) < rotation_speed) ? angle_difference : (angle_difference > 0 ? rotation_speed : -rotation_speed);

            double climb_rate = 8.0 * delta_time;
            if (std::abs(dz) < climb_rate) {
                posz_ = target_waypoint_.z;
            } else {
                posz_ += (dz > 0 ? climb_rate : -climb_rate);
            }
           //posz_ += (std::abs(dz) < climb_rate) ? dz : (dz > 0 ? climb_rate : -climb_rate);
        }
    }

    double distance = speed_ * delta_time;

    posx_ += distance * std::cos(bearing_);
    posy_ += distance * std::sin(bearing_);
}

