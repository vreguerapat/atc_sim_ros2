#include "avion.hpp"
#include <cmath>

Avion::Avion() 
{
    id_ = generateRandomID();
    airline_ = "Iberia";
    posx_ = generateRandomPosition(true);
    posy_ = generateRandomPosition(true);
    posz_ = generateRandomPosition(false);
    speed_ = 10.0;
    bearing_ =generateRandomBearing();
    elevation_angle_ = 0.0;
    reached_waypoint_ = false;
}

std::string Avion::getID() const { return id_; }
std::string Avion::getAirline() const {return airline_; }
double Avion::getPosX() const { return posx_; }
double Avion::getPosY() const { return posy_; }
double Avion::getPosZ() const { return posz_; }
double Avion::getSpeed() const { return speed_; }
double Avion::getBearing() const { return bearing_; }
double Avion::getElevationAngle() const { return elevation_angle_; ;}

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

// Metodo para actualizar la posicion
void Avion::update(double delta_time, double waypoint_x, double waypoint_y, double waypoint_z)
{
    // Primero se calcula el angulo hacia el waypoint
    double dx = waypoint_x - posx_;
    double dy = waypoint_y - posy_;
    double dz = waypoint_z - posz_;
    double distance_to_waypoint = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (!reached_waypoint_){
        // Margen de distancia al waypoint
        double waypoint_threshold = 0.2;

        if (distance_to_waypoint < waypoint_threshold) {
            reached_waypoint_ = true;
        } else {
            // Bearing objetivo
            double target_bearing = std::atan2(dy, dx);

            // Angulo elevacion
            double distance_xy = std::sqrt(dx * dx + dy * dy);
            double target_elevation_angle = std::atan2(dz, distance_xy);

            // Se establece un valor max para el angulo
            double max_elevation_angle = M_PI / 3;
            target_elevation_angle = std::max(-max_elevation_angle, std::min(target_elevation_angle,max_elevation_angle));
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

            double elevation_angle_difference = target_elevation_angle - elevation_angle_;
            double elevation_speed = 1;

            if (std::abs(elevation_angle_difference) < elevation_speed) {
                elevation_angle_ = target_elevation_angle;
            } else {
                elevation_angle_ += (elevation_angle_difference > 0 ? elevation_speed : -elevation_speed);
            }

        }

    }

    double distance = speed_ * delta_time;

    posx_ += distance * std::cos(bearing_) * std::cos(elevation_angle_);
    posy_ += distance * std::sin(bearing_) * std::cos(elevation_angle_);
    posz_ += distance * std::sin(elevation_angle_);
}

