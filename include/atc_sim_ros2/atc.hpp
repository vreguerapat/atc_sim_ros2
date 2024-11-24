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
    int contador_colisiones_;

    atc_sim_ros2::msg::ListaAviones::SharedPtr aviones_lista_;

    rclcpp::TimerBase::SharedPtr collision_check_timer_;
    rclcpp::TimerBase::SharedPtr route_management_timer_;

    void avionesCallback(const atc_sim_ros2::msg::ListaAviones::SharedPtr msg);
    void manageRoutes(const atc_sim_ros2::msg::ListaAviones::SharedPtr lista_aviones);
    void assignRoute(atc_sim_ros2::msg::Flight& avion_msg);
    bool willCollidePredictive(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2, double time_ahead, double threshold_distance);
    bool checkRouteCollision(const atc_sim_ros2::msg::Flight& avion_msg, const std::vector<atc_sim_ros2::msg::Waypoint>& new_route);
    void adjustRoute(atc_sim_ros2::msg::Flight& avion_msg);
    int selectLeastAssignedDestination();
    std::vector<atc_sim_ros2::msg::Waypoint> generateIntermediateWaypoints(const atc_sim_ros2::msg::Waypoint& start, const atc_sim_ros2::msg::Waypoint& end, int num_points);
    void checkCollisions(atc_sim_ros2::msg::ListaAviones::SharedPtr msg);
    bool areTooClose(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2, double threshold_distance);
    bool areRoutesIntersecting(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2);
    double calculateTimeToWaypoint(const atc_sim_ros2::msg::Flight& avion, const atc_sim_ros2::msg::Waypoint& wp) ;
    void adjustTrajectory(atc_sim_ros2::msg::Flight& avion_msg, atc_sim_ros2::msg::Flight& otro_avion);
    void adjustAltitud(atc_sim_ros2::msg::Flight& avion_msg);
    void adjustNextWaypoint(atc_sim_ros2::msg::Flight& avion_msg);
    void adjustSpeed(atc_sim_ros2::msg::Flight& avion1, atc_sim_ros2::msg::Flight& avion2);
    double calculateBearing(atc_sim_ros2::msg::Flight& from, atc_sim_ros2::msg::Flight& to);
    double normalizeAngle(double angle);
    double calculateDistance(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2);
};

#endif