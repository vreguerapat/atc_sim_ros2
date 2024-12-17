#ifndef AEROPUERTO_HPP_
#define AEROPUERTO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include "avion.hpp"
#include "atc_sim_ros2/msg/waypoint_update.hpp"
#include <vector>
#include <chrono>
#include <memory>
#include <array>
#include <cmath>

class Aeropuerto : public rclcpp::Node {
    public:
        Aeropuerto();
        void agregarAvion();
        double generateRandomCoordinate();
        double generateRandomCoordinateX();
        double generateRandomCoordinateY();
        double generateRandomAltitude();
        double calculateDistance(const Avion& avion1, const Avion& avion2);
        void updateWaypoints(const atc_sim_ros2::msg::WaypointUpdate& update_msg);

    private:
        rclcpp::Publisher<atc_sim_ros2::msg::ListaAviones>::SharedPtr lista_aviones_publisher_;
        rclcpp::Subscription<atc_sim_ros2::msg::WaypointUpdate>::SharedPtr waypoints_sub_;
        rclcpp::TimerBase::SharedPtr update_timer_;
        rclcpp::TimerBase::SharedPtr avion_timer_;
        std::vector<Avion> lista_aviones_;
        std::vector<std::array<float,3>> waypoints_;
        int lado;
        int aviones_totales;


        void update_airport(double delta_time);
    
};

#endif