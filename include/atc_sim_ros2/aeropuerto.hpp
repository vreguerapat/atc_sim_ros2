#ifndef AEROPUERTO_HPP_
#define AEROPUERTO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include "avion.hpp"
#include <vector>
#include <chrono>
#include <memory>
#include <array>

class Aeropuerto : public rclcpp::Node {
    public:
        Aeropuerto();
        void agregarAvion();

    private:
        rclcpp::Publisher<atc_sim_ros2::msg::ListaAviones>::SharedPtr lista_aviones_publisher_;
        rclcpp::TimerBase::SharedPtr update_timer_;
        rclcpp::TimerBase::SharedPtr avion_timer_;
        std::vector<Avion> lista_aviones_;
        std::vector<std::array<float,3>> waypoints_;


        void update_airport(double delta_time);
    
};

#endif