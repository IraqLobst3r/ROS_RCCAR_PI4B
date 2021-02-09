#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using WeatherData = std::array<std_msgs::msg::String, 3>;
using namespace std::chrono_literals;

class Weatherman : public rclcpp::Node {
   public:
    Weatherman() : Node("Weatherman") {
        publisher_ =
            this->create_publisher<std_msgs::msg::String>("weather", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&Weatherman::timer_callback, this));
    }

   private:
    void timer_callback() {
        std::random_shuffle(data.begin(), data.end());
        RCLCPP_INFO(this->get_logger(), "Weather: '%s'",
                    this->data[0].data.c_str());
        publisher_->publish(data[0]);
    }

    WeatherData generate_weather_data() {
        std_msgs::msg::String msg1, msg2, msg3;
        msg1.data = "It's gonna be cloudy today!";
        msg2.data = "Rain everywhere!";
        msg3.data = "Sunny!";
        return {msg1, msg2, msg3};
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    WeatherData data = this->generate_weather_data();
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Weatherman>());
    rclcpp::shutdown();
    return 0;
}
