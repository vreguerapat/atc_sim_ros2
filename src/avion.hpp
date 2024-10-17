#ifndef AVION_HPP
#define AVION_HPP

#include <string>
#include <cstdlib>
#include <ctime>

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
        double getElevationAngle() const;

        void update(double delta_time, double waypoint_x, double waypoint_y, double waypoint_z); //Método para actualizar posicion aviones

    private:
        std::string id_;
        std::string airline_;
        double posx_, posy_, posz_, speed_, bearing_, elevation_angle_;
        bool reached_waypoint_;

        std::string generateRandomID();
        double generateRandomPosition(bool margins);
        double generateRandomBearing();
};

#endif