#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/lista_aviones.hpp"
#include "avion.hpp"
#include <vector>
#include <chrono>
#include <memory>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class Aeropuerto : public rclcpp::Node
{
    public:
        Aeropuerto() : Node("aeropuerto")
        {
            //Se publica el mensaje lista_aviones
            lista_aviones_publisher_ = this->create_publisher<atc_sim_ros2::msg::ListaAviones>("lista_aviones",10);
            pose_array_pub_= this->create_publisher<geometry_msgs::msg::PoseArray>("pose_topic", 10);

            update_timer_ = this->create_wall_timer( 1s, [this]() { update_airport(0.01); });

            show_timer_ = this->create_wall_timer(1s, [this]() {show_airport();});

            avion_timer_= this->create_wall_timer(5s, [this]() {agregarAvion();});
        }

        //Funcion para agregar nuevo avion
        void agregarAvion()
        {
            Avion nuevo_avion;
            lista_aviones_.push_back(nuevo_avion);
            RCLCPP_INFO(this->get_logger(), "Se agregó un nuevo avion");
        }
    
    private:
        //Variables privadas
        rclcpp::Publisher<atc_sim_ros2::msg::ListaAviones>::SharedPtr lista_aviones_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
        rclcpp::TimerBase::SharedPtr update_timer_;
        rclcpp::TimerBase::SharedPtr show_timer_;
        rclcpp::TimerBase::SharedPtr avion_timer_;
        std::vector<Avion> lista_aviones_;

        void update_airport(double delta_time)
        {
            for(auto &avion : lista_aviones_){
                avion.update(delta_time);
            }
        }

        void show_airport()
        {
            //Hay que crear el mensaje del nuevo avion
            atc_sim_ros2::msg::ListaAviones msg_lista;

            geometry_msgs::msg::PoseArray pose_array;
            pose_array.header.frame_id = "map";
            pose_array.header.stamp = this->now();

            RCLCPP_INFO(this->get_logger(), "Publicando lista de %zu aviones", lista_aviones_.size());
            
            for (const auto &avion : lista_aviones_){
                atc_sim_ros2::msg::Flight avion_msg;
                avion_msg.id = avion.getID();
                avion_msg.airline = avion.getAirline();
                avion_msg.posx = avion.getPosX();
                avion_msg.posy = avion.getPosY();
                avion_msg.posz = avion.getPosZ();
                avion_msg.speed = avion.getSpeed();
                avion_msg.bearing = avion.getBearing(); 

                //Se añade el avion a la lista
                msg_lista.aviones.push_back(avion_msg);

                //Para visualizar en RViz
                geometry_msgs::msg::Pose pose_msg;
            
                pose_msg.position.x = avion.getPosX();
                pose_msg.position.y = avion.getPosY();
                pose_msg.position.z = avion.getPosZ();

                tf2::Quaternion q;
                q.setRPY(0, 0, avion.getBearing());
                pose_msg.orientation.x = q.x();
                pose_msg.orientation.y = q.y();
                pose_msg.orientation.z = q.z();
                pose_msg.orientation.w = q.w();

                pose_array.poses.push_back(pose_msg);


                std::cout << "ID: " << avion_msg.id
                          << ", Airline: " << avion_msg.airline
                          << ", PosX: " << avion_msg.posx
                          << ", PosY: " << avion_msg.posy
                          << ", PosZ: " << avion_msg.posz
                          << ", Speed: " << avion_msg.speed
                          << std::endl;
            }

            pose_array_pub_->publish(pose_array);
            lista_aviones_publisher_->publish(msg_lista);
        }


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto aeropuerto = std::make_shared<Aeropuerto>();
    rclcpp::spin(aeropuerto);
    rclcpp::shutdown();
    return 0;
}