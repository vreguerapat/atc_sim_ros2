#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

using namespace std::chrono_literals;

class Visualizador : public rclcpp::Node{
    public:
        Visualizador() : Node("visualizador")
        {
            // Se suscribe al topic lista_aviones
            aviones_subscriber_ = this->create_subscription<atc_sim_ros2::msg::ListaAviones>(
                "lista_aviones", 10, std::bind(&Visualizador::visualizar_aviones, this, std::placeholders::_1));
            
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("aviones_marker", 10);
        }
    private:
        rclcpp::Subscription<atc_sim_ros2::msg::ListaAviones>::SharedPtr aviones_subscriber_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

        void visualizar_aviones(const atc_sim_ros2::msg::ListaAviones::SharedPtr msg_lista)
        {
            // Se crea un marker array
            visualization_msgs::msg::MarkerArray marker_array;
           int id = 0;
            

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
                q.setRPY( 0, avion_msg.bearing, avion_msg.elevation_angle);
                marker.pose.orientation.x = q.x();
                marker.pose.orientation.y = q.y();
                marker.pose.orientation.z = q.z();
                marker.pose.orientation.w = q.w();
                marker_array.markers.push_back(marker);
                
            }

            // Se crea un waypoint, cilindro verde
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
            waypoint_marker.pose.position.x = 0.0;
            waypoint_marker.pose.position.y = 0.0;
            waypoint_marker.pose.position.z = 5.0;;
            waypoint_marker.lifetime = rclcpp::Duration(1s);

            marker_array.markers.push_back(waypoint_marker);


            RCLCPP_INFO(this->get_logger(), "Publicando %zu markers", marker_array.markers.size());
            RCLCPP_INFO(this->get_logger(), "Recibidos %zu aviones", msg_lista->aviones.size());
            marker_pub_->publish(marker_array);
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto visualizador = std::make_shared<Visualizador>();
    rclcpp::spin(visualizador);
    rclcpp::shutdown();
    return 0;
}