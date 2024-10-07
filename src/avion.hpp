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

    private:
        std::string id_;
        std::string airline_;
        double posx_, posy_, posz_, speed_;

        std::string generateRandomID();
        double generateRandomPosition();
};

#endif