#ifndef AVION_HPP
#define AVION_HPP

#include <string>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <array>
#include "atc_sim_ros2/msg/waypoint.hpp"
#include "atc_sim_ros2/msg/flight.hpp"

class Avion {
    public:
        Avion();

        Avion(const atc_sim_ros2::msg::Flight& avion_msg) {
            id_ = avion_msg.id;
            posx_ = avion_msg.posx;
            posy_ = avion_msg.posy;
            posz_ = avion_msg.posz;
            airline_ = avion_msg.airline;
            speed_ = avion_msg.speed;
            bearing_ = avion_msg.bearing;
        }
        std::string getID() const;
        std::string getAirline() const;
        double getPosX() const;
        double getPosY() const;
        double getPosZ() const;
        double getSpeed() const;
        double getBearing() const;

        void addWaypoints(const std::vector<atc_sim_ros2::msg::Waypoint>& waypoints);
        void clearWaypoints();
        void setSpeed(double speed);
        void setPosX(double x);
        void setPosY(double y);
        void setPosZ(double z);
        void setBearing(double bearing);
        const std::vector<atc_sim_ros2::msg::Waypoint>& getWaypoints() const;

        bool getRutaCompletada() const {return ruta_completada_; }
        void setRutaCompletada(bool completada) { ruta_completada_ = completada; }
        
        void selectRandomWaypoint() ;
        static std::vector<atc_sim_ros2::msg::Waypoint> generateIntermediateWaypoints(const atc_sim_ros2::msg::Waypoint& start, const atc_sim_ros2::msg::Waypoint& end, int num_points) ;
        void update(double delta_time); //Método para actualizar posicion aviones

    private:
        std::string id_;
        std::string airline_;
        double posx_, posy_, posz_, speed_, bearing_;
        bool reached_waypoint_;
        bool ruta_completada_;
        atc_sim_ros2::msg::Waypoint target_waypoint_;

        std::vector<atc_sim_ros2::msg::Waypoint> waypoints_;

        std::string generateRandomID();
        double generateRandomPosition(bool margins, bool isAltitud);
        double generateBearing();
        std::string assignRandomAirline();
};

#endif