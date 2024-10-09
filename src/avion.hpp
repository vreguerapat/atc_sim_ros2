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

        void update(double delta_time); //Método para actualizar posicion aviones

    private:
        std::string id_;
        std::string airline_;
        double posx_, posy_, posz_, speed_, bearing_;

        std::string generateRandomID();
        double generateRandomPosition();
        double generateRandomBearing();
};

#endif