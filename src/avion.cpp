#include "avion.hpp"
#include <cmath>
#include "rclcpp/rclcpp.hpp"

Avion::Avion() 
{
    id_ = generateRandomID();
    airline_ = "Iberia";
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


//Funcion que genera de manera aleatoria el ID entre 1000 - 9999
std::string Avion::generateRandomID()
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    int random_id = 1000 + std::rand() % 9000;
    return std::to_string(random_id);
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
void Avion::selectRandomWaypoint(const std::vector<std::array<float, 3>>& waypoints)
{
    int selected_waypoint = rand() % waypoints.size();
    target_waypoint_ = waypoints[selected_waypoint];

    RCLCPP_INFO(rclcpp::get_logger("avion_logger"), "Avion %s dirigiendose al waypoint %d", id_.c_str(), selected_waypoint);
}

// Metodo para actualizar la posicion
void Avion::update(double delta_time)
{
    // Primero se calcula el angulo hacia el waypoint
    double dx = target_waypoint_[0] - posx_;
    double dy = target_waypoint_[1] - posy_;
    double dz = target_waypoint_[2] - posz_;
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
                posz_ = target_waypoint_[2];
            } else {
                posz_ += (dz > 0 ? climb_rate : -climb_rate);
            }


        }

    }

    double distance = speed_ * delta_time;

    posx_ += distance * std::cos(bearing_);
    posy_ += distance * std::sin(bearing_);
}

