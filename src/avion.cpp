#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "atc_sim_ros2/msg/flight.hpp"

#include <string>
#include <chrono>
#include <cstdlib>
#include <ctime>

using namespace std::chrono_literals;

class Avion : public rclcpp::Node
{
    public:
        Avion(const std::string &id) : Node("avion_" + id), id_(id)
        {
            avion_ = this->create_publisher<atc_sim_ros2::msg::Flight>("flight_topic", 10);

            auto publish_msg = [this]() -> void {
                auto message = atc_sim_ros2::msg::Flight();

                message.id = id_;
                message.airline = "Iberia";
 
                std::cout << "Publishing Flight Info\nID: " <<message.id << " Airline: " << message.airline << std::endl;

                this->avion_->publish(message);
            };
            timer_ = this->create_wall_timer(1s, publish_msg);
        }
    private:
        rclcpp::Publisher<atc_sim_ros2::msg::Flight>::SharedPtr avion_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string id_;
        std::function<void()> publish_msg;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    std::string id = std::to_string(std::rand() % 1000);

    auto avion = std::make_shared<Avion>(id);
    rclcpp::spin(std::make_shared<Avion>(id));
    rclcpp::shutdown();

    return 0;
}