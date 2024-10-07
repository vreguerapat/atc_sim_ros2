#include "avion.hpp"


Avion::Avion() 
{
    id_ = generateRandomID();
    airline_ = "Iberia";
    posx_ = generateRandomPosition();
    posy_ = generateRandomPosition();
    posz_ = generateRandomPosition();
    speed_ = 500.0;
}

std::string Avion::getID() const { return id_; }
std::string Avion::getAirline() const {return airline_; }
double Avion::getPosX() const { return posx_; }
double Avion::getPosY() const { return posy_; }
double Avion::getPosZ() const { return posz_; }
double Avion::getSpeed() const { return speed_; }

//Funcion que genera de manera aleatoria el ID entre 1000 - 9999
std::string Avion::generateRandomID()
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    int random_id = 1000 + std::rand() % 9000;
    return std::to_string(random_id);
}

//Funcion que genera de manera aleatoria un valor entre -1 y 1
double Avion::generateRandomPosition()
{
    return -1.0 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 2.0));
}
