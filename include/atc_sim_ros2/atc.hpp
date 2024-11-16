#ifndef ATC_HPP
#define ATC_HPP

#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/flight.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include "atc_sim_ros2/avion.hpp"
#include "atc_sim_ros2/msg/waypoint.hpp"
#include "atc_sim_ros2/msg/waypoint_update.hpp"
#include <vector>
#include <cmath>
#include <string>

class ATC : public rclcpp::Node {
    public:
    ATC();

    private:
    rclcpp::Subscription<atc_sim_ros2::msg::ListaAviones>::SharedPtr aviones_sub_;
    rclcpp::Publisher<atc_sim_ros2::msg::WaypointUpdate>::SharedPtr waypoints_pub_;
    std::vector<atc_sim_ros2::msg::Waypoint> waypoints_destino_;
    std::vector<int> contador_destinos_; 

    void manageRoutes(const atc_sim_ros2::msg::ListaAviones::SharedPtr lista_aviones);
    void assignRoute(atc_sim_ros2::msg::Flight& avion_msg);
    int selectLeastAssignedDestination();
    std::vector<atc_sim_ros2::msg::Waypoint> generateIntermediateWaypoints(const atc_sim_ros2::msg::Waypoint& start, const atc_sim_ros2::msg::Waypoint& end, int num_points);
    void checkCollisions(const atc_sim_ros2::msg::ListaAviones::SharedPtr msg);
    double calculateDistance(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2);
};

#endif