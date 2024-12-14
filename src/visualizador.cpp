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
        {0.0, 0.0, 0.0},
        {0.0, 15.0, 5.0},
        {10.0, 25.0, 7.0},
        {40.0, 20.0, 5.0},
        {37.0, 30.0, 5.0},
        {30.0, 35.0, 5.0},
        {20.0, 30.0, 5.0},
        {17.0, 20.0, 5.0},
        {20.0, 10.0, 5.0},
        {30.0, 5.0, 5.0},
        {37.0, 10.0, 5.0}
    };
   
}

void Visualizador::publicar_waypoints()
{
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

        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = this->now();
        text_marker.ns = "wp_texto";
        text_marker.id = id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.scale.z = 0.5;
        text_marker.color.a = 1.0;
        text_marker.color.r =1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.lifetime = rclcpp::Duration(1s);
        text_marker.pose.position.x = wp[0];
        text_marker.pose.position.y = wp[1];
        text_marker.pose.position.z = wp[2] + 0.5;
        text_marker.text = "[" + std::to_string(wp[0]) + ", " + std::to_string(wp[1]) + ", " + std::to_string(wp[2]) + "]";
        marker_array.markers.push_back(text_marker);
    }
    waypoint_marker_publisher_->publish(marker_array);

    
}

void Visualizador::visualizar_aviones(const atc_sim_ros2::msg::ListaAviones::SharedPtr msg_lista)
{
    // Se crea un marker array
    visualization_msgs::msg::MarkerArray marker_array;

    for (int i = 0; i < 1000; i++) {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = "map";
        delete_marker.header.stamp = this->now();
        delete_marker.ns = "aviones";
        delete_marker.id = i;
        delete_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(delete_marker);
    }
    marker_pub_->publish(marker_array);
    marker_array.markers.clear();
    
    int id = 0;
            
    for (const auto &avion_msg : msg_lista->aviones)
    {
        RCLCPP_INFO(this->get_logger(), "Avion ID: %s | Ruta completada: %s | Waypoints vacios: %s", avion_msg.id.c_str(), avion_msg.ruta_completada ? "Sí" : "No", avion_msg.waypoints.empty() ? "Sí" : "No");
        
        // Marcado Avion, Flecha
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

        // Visualizar waypoints intermedios
        for (const auto& wp : avion_msg.waypoints) {
            visualization_msgs::msg::Marker waypoint_marker;
            waypoint_marker.header.frame_id = "map";
            waypoint_marker.header.stamp = this->now();
            waypoint_marker.ns = "waypoints_intermedios";
            waypoint_marker.id = id++;
            waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
            waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
            waypoint_marker.scale.x = 0.2;
            waypoint_marker.scale.y = 0.2;
            waypoint_marker.scale.z = 0.2;
            waypoint_marker.color.a = 1.0;
            waypoint_marker.color.r = 0.0;
            waypoint_marker.color.g = 0.0;
            waypoint_marker.color.b = 1.0;
            waypoint_marker.pose.position.x = wp.x;
            waypoint_marker.pose.position.y = wp.y;
            waypoint_marker.pose.position.z = wp.z;
            waypoint_marker.lifetime = rclcpp::Duration(1s);
            marker_array.markers.push_back(waypoint_marker);
        }

        if (!avion_msg.ruta_completada && !avion_msg.waypoints.empty()) {
            visualization_msgs::msg::Marker line_strip;
            line_strip.header.frame_id = "map";
            line_strip.header.stamp = this->now();
            line_strip.ns = "waypoints_lines";
            line_strip.id = id++;
            line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_strip.scale.x = 0.1;
            line_strip.color.a = 0.5;
            line_strip.color.r = 0.0;
            line_strip.color.g = 0.0;
            line_strip.color.b = 1.0;
            line_strip.lifetime = rclcpp::Duration(1s);
            
            // Posicion del avion
            geometry_msgs::msg::Point p;
            p.x = avion_msg.posx;
            p.y = avion_msg.posy;
            p.z = avion_msg.posz;
            line_strip.points.push_back(p);

            for (const auto& wp : avion_msg.waypoints) {
                geometry_msgs::msg::Point point;
                point.x = wp.x;
                point.y = wp.y;
                point.z = wp.z;
                line_strip.points.push_back(point);
            }

            marker_array.markers.push_back(line_strip);

        } else {
            RCLCPP_INFO(this->get_logger(), "El avion %s ha completado su ruta o no tiene waypoints", avion_msg.id.c_str());
        }
       
    }

    RCLCPP_INFO(this->get_logger(), "Recibidos %zu aviones", msg_lista->aviones.size());
    marker_pub_->publish(marker_array);
}
