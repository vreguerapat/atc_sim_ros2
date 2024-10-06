#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/flight.hpp"

#include <string>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <random>

using namespace std::chrono_literals;

class Avion : public rclcpp::Node
{
    public:
        Avion() : Node("avion_" + generateRandomID()), id_(generateRandomID()), posx_(generateRandomPosition()), posy_(generateRandomPosition()), posz_(generateRandomPosition())
        {
            avion_ = this->create_publisher<atc_sim_ros2::msg::Flight>("flight_topic", 10);

            auto publish_msg = [this]() -> void {
                auto message = atc_sim_ros2::msg::Flight();

                message.id = id_;
                message.airline = "Iberia";
                message.posx = posx_;
                message.posy = posy_;
                message.posz = posz_;
                message.speed = 500.0;
 
                std::cout << "Publishing Flight Info\nID: " <<message.id << "\nAirline: " << message.airline << "\nPosX: " << message.posx << " PosY: " << message.posy << " PosZ: " << message.posz << "\nSpeed: " << message.speed << std::endl;

                this->avion_->publish(message);
            };
            timer_ = this->create_wall_timer(1s, publish_msg);
        }
    private:
        //Funcion para generar ID aleatoria entre 1000 y 9999
        std::string generateRandomID()
        {
            std::srand(static_cast<unsigned int>(std::time(nullptr)));
            int random_id = 1000 + std::rand() % 9000;
            return std::to_string(random_id);
        }

        //Funcion para genera posiciones aleatorias entr -1 y 1
        double generateRandomPosition()
        {
            return -1.0 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 2.0));
        }

        rclcpp::Publisher<atc_sim_ros2::msg::Flight>::SharedPtr avion_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string id_;
        double posx_, posy_, posz_;
        std::function<void()> publish_msg;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto avion = std::make_shared<Avion>();
    rclcpp::spin(avion);
    rclcpp::shutdown();

    return 0;
}