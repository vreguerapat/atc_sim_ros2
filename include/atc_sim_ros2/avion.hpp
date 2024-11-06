#ifndef AVION_HPP
#define AVION_HPP

#include <string>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <array>
#include "atc_sim_ros2/msg/waypoint.hpp"

class Avion {
    public:
        Avion();

        std::string getID() const;
        std::string getAirline() const;
        double getPosX() const;
        double getPosY() const;
        double getPosZ() const;
        double getSpeed() const;
        double getBearing() const;

        void addWaypoints(const std::vector<atc_sim_ros2::msg::Waypoint>& waypoints);
        const std::vector<atc_sim_ros2::msg::Waypoint>& getWaypoints() const;
        
        void selectRandomWaypoint() ;
        std::vector<atc_sim_ros2::msg::Waypoint> generateIntermediateWaypoints(const atc_sim_ros2::msg::Waypoint& start, const atc_sim_ros2::msg::Waypoint& end, int num_points) ;
        void update(double delta_time); //Método para actualizar posicion aviones

    private:
        std::string id_;
        std::string airline_;
        double posx_, posy_, posz_, speed_, bearing_;
        bool reached_waypoint_;
        atc_sim_ros2::msg::Waypoint target_waypoint_;

        std::vector<atc_sim_ros2::msg::Waypoint> waypoints_;

        std::string generateRandomID();
        double generateRandomPosition(bool margins, bool isAltitud);
        double generateBearing();
        std::string assignRandomAirline();
};

#endif