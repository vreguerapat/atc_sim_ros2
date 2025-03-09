#ifndef VISUALIZADOR_HPP_
#define VISUALIZADOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <vector>
#include <array>

using namespace std::chrono_literals;

class Visualizador : public rclcpp::Node{
    public:
        Visualizador();
        ~Visualizador() override;

    private:
        rclcpp::Subscription<atc_sim_ros2::msg::ListaAviones>::SharedPtr aviones_subscriber_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<std::array<float,11>> waypoints_; 

        void publicar_waypoints();
        void visualizar_aviones(const atc_sim_ros2::msg::ListaAviones::SharedPtr msg_lista);
        

};

#endif 