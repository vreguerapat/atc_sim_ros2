#ifndef AVION_HPP
#define AVION_HPP

#include <string>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <array>

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
        
        void selectRandomWaypoint(const std::vector<std::array<float, 3>>& waypoints) ;
        void update(double delta_time); //Método para actualizar posicion aviones

    private:
        std::string id_;
        std::string airline_;
        double posx_, posy_, posz_, speed_, bearing_;
        bool reached_waypoint_;
        std::array<float, 3> target_waypoint_;

        std::string generateRandomID();
        double generateRandomPosition(bool margins);
        double generateRandomBearing();
        std::string assignRandomAirline();
};

#endif