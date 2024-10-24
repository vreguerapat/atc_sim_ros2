#include "atc_sim_ros2/visualizador.hpp"
#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

Visualizador::Visualizador() : Node("visualizador")
{
    aviones_subscriber_ = this->create_subscription<atc_sim_ros2::msg::ListaAviones>(
        "lista_aviones", 10, std::bind(&Visualizador::visualizar_aviones, this, std::placeholders::_1));
            
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("aviones_marker", 10);
    waypoint_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_marker", 10);
    timer_ = this->create_wall_timer( 1s, [this]() {publicar_waypoints();});
        
    waypoints_ = {
        {0.0, 0.0, 5.0},
        {5.0, 5.0, 5.0}
    };
}

void Visualizador::publicar_waypoints()
{
    RCLCPP_INFO(this->get_logger(), "Publicando waypoints");
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 999;
    for (const auto& wp : waypoints_){
        visualization_msgs::msg::Marker waypoint_marker;
        waypoint_marker.header.frame_id = "map";
        waypoint_marker.header.stamp = this->now();
        waypoint_marker.ns = "waypoints";
        waypoint_marker.id = id++;
        waypoint_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
        waypoint_marker.scale.x = 0.5;
        waypoint_marker.scale.y = 0.5;
        waypoint_marker.scale.z = 0.5;
        waypoint_marker.color.a = 1.0;
        waypoint_marker.color.g = 1.0;
        waypoint_marker.color.r = 0.0;
        waypoint_marker.color.b = 0.0;
        waypoint_marker.pose.position.x = wp[0];
        waypoint_marker.pose.position.y = wp[1];
        waypoint_marker.pose.position.z = wp[2];
        waypoint_marker.lifetime = rclcpp::Duration(1s);

        marker_array.markers.push_back(waypoint_marker);
    }
    waypoint_marker_publisher_->publish(marker_array);
}

void Visualizador::visualizar_aviones(const atc_sim_ros2::msg::ListaAviones::SharedPtr msg_lista)
{
    // Se crea un marker array
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    publicar_waypoints();
            
    for (const auto &avion_msg : msg_lista->aviones)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "aviones";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 1.0;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = rclcpp::Duration(1s);

        marker.pose.position.x = avion_msg.posx;
        marker.pose.position.y = avion_msg.posy;
        marker.pose.position.z = avion_msg.posz;
        tf2::Quaternion q;
        q.setRPY( 0, 0, avion_msg.bearing);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker_array.markers.push_back(marker);

        // Marcador de texto para id avion
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = this->now();
        text_marker.ns = "aviones_texto";
        text_marker.id = id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.scale.z = 0.5;
        text_marker.color.a = 1.0;
        text_marker.color.r =1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.lifetime = rclcpp::Duration(1s);

        text_marker.pose.position.x = avion_msg.posx;
        text_marker.pose.position.y = avion_msg.posy;
        text_marker.pose.position.z = avion_msg.posz + 0.5;
        text_marker.text = avion_msg.id;
        marker_array.markers.push_back(text_marker);

         
                
    }


    RCLCPP_INFO(this->get_logger(), "Publicando %zu markers", marker_array.markers.size());
    RCLCPP_INFO(this->get_logger(), "Recibidos %zu aviones", msg_lista->aviones.size());
    marker_pub_->publish(marker_array);
}
