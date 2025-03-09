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
    ~ATC() override;

    struct IntersectionResult {
        double t1, t2;
        double x, y, z;
    };

    std::optional<IntersectionResult> intersectLines3D(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2, double threshold_distance);

    private:
    rclcpp::Subscription<atc_sim_ros2::msg::ListaAviones>::SharedPtr aviones_sub_;
    rclcpp::Publisher<atc_sim_ros2::msg::WaypointUpdate>::SharedPtr waypoints_pub_;
    std::vector<atc_sim_ros2::msg::Waypoint> waypoints_destino_;
    std::vector<int> contador_destinos_; 
    int contador_colisiones_;
    bool landing32L;
    bool landing32R;
    std::vector<atc_sim_ros2::msg::Waypoint> circuit1_; 
    std::vector<atc_sim_ros2::msg::Waypoint> circuit2_;
    std::vector<atc_sim_ros2::msg::Waypoint> landing_route32L_; 
    std::vector<atc_sim_ros2::msg::Waypoint> landing_route32R_; 

    atc_sim_ros2::msg::ListaAviones::SharedPtr aviones_lista_;

    rclcpp::TimerBase::SharedPtr collision_check_timer_;
    rclcpp::TimerBase::SharedPtr speed_check_timer_;
    rclcpp::TimerBase::SharedPtr route_management_timer_;

    void avionesCallback(const atc_sim_ros2::msg::ListaAviones::SharedPtr msg);
    void checkSpeed(const atc_sim_ros2::msg::ListaAviones::SharedPtr aviones_lista);
    void manageRoutes(const atc_sim_ros2::msg::ListaAviones::SharedPtr lista_aviones);
    void assignRoute(atc_sim_ros2::msg::Flight& avion_msg);
    const std::vector<atc_sim_ros2::msg::Waypoint>& findCircuit(const atc_sim_ros2::msg::Flight& avion_msg);
    void assignLandingRoute32L(atc_sim_ros2::msg::Flight& avion_msg);
    void assignLandingRoute32R(atc_sim_ros2::msg::Flight& avion_msg);
    std::vector<atc_sim_ros2::msg::Waypoint> generateRoute(const atc_sim_ros2::msg::Waypoint start_waypoint, atc_sim_ros2::msg::Flight& avion_msg, const std::vector<atc_sim_ros2::msg::Waypoint>& circuit);
    atc_sim_ros2::msg::Waypoint findClosestWaypoint(double pos_x, double pos_y, double pos_z);
    void adjustRoute(atc_sim_ros2::msg::Flight& avion_msg);
    int selectLeastAssignedDestination();
    std::vector<atc_sim_ros2::msg::Waypoint> generateIntermediateWaypoints(const atc_sim_ros2::msg::Waypoint& start, const atc_sim_ros2::msg::Waypoint& end, int num_points);
    void checkCollisions(atc_sim_ros2::msg::ListaAviones::SharedPtr msg);
    bool routesTooClose(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2, double threshold_distance);
    std::vector<atc_sim_ros2::msg::Waypoint> generateTrajectoryPoints(const atc_sim_ros2::msg::Flight& avion, const atc_sim_ros2::msg::Waypoint& waypoint, int num_points);
    bool areTooClose(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2, double threshold_distance);
    bool areRoutesIntersecting(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2);
    double calculateTimeToWaypoint(const atc_sim_ros2::msg::Flight& avion, double x, double y, double z) ;
    bool checkWaypointInCircuitOrClose(const atc_sim_ros2::msg::Flight& avion);
    bool checkWaypointExit(const atc_sim_ros2::msg::Flight& avion);
    void adjustTrajectory(atc_sim_ros2::msg::Flight& avion_msg, atc_sim_ros2::msg::Flight& otro_avion);
    bool checkCollisionAfterAdjustment(atc_sim_ros2::msg::Flight& avion1, atc_sim_ros2::msg::Flight& avion2);
    void adjustAltitude(atc_sim_ros2::msg::Flight& avion1, atc_sim_ros2::msg::Flight& avion2);
    void adjustNextWaypoint(atc_sim_ros2::msg::Flight& avion1, atc_sim_ros2::msg::Flight& avion2);
    atc_sim_ros2::msg::Waypoint createLateralDisplacedWaypoint(atc_sim_ros2::msg::Flight& avion, bool moveRight);
    void adjustSpeed(atc_sim_ros2::msg::Flight& avion1, atc_sim_ros2::msg::Flight& avion2);
    double calculateBearing(atc_sim_ros2::msg::Flight& from, atc_sim_ros2::msg::Flight& to);
    double normalizeAngle(double angle);
    double calculateDistance(const atc_sim_ros2::msg::Flight& avion1, const atc_sim_ros2::msg::Flight& avion2);
    double distanceBetweenWaypoints( const atc_sim_ros2::msg::Waypoint& wp1, const atc_sim_ros2::msg::Waypoint& wp2);

};

#endif