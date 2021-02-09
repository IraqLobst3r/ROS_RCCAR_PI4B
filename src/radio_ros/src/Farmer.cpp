#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/music.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using feedbackList = std::array<std_msgs::msg::String, 3>;
using std::placeholders::_1;

class Farmer : public rclcpp::Node {
   public:
    Farmer() : Node("Farmer") {
        publisher_ =
            this->create_publisher<std_msgs::msg::String>("feedback", 10);

        music_sub_ = this->create_subscription<custom_interfaces::msg::Music>(
            "music", 10, std::bind(&Farmer::music_callback, this, _1));

        weather_sub_ = this->create_subscription<std_msgs::msg::String>(
            "weather", 10, std::bind(&Farmer::weather_callback, this, _1));
    }

   private:
    void music_callback(const custom_interfaces::msg::Music::SharedPtr msg) {
        std::random_shuffle(data.begin(), data.end());
        RCLCPP_INFO(this->get_logger(), "Music: '%s'", msg->title.c_str());
        RCLCPP_INFO(this->get_logger(), "Feedback: '%s'", data[0].data.c_str());
        publisher_->publish(data[0]);
    }

    void weather_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Weather: '%s'", msg->data.c_str());
    }

    feedbackList generate_feedback_list() {
        std_msgs::msg::String msg1, msg2, msg3;
        msg1.data = "good";
        msg2.data = "OK";
        msg3.data = "mhh";

        return {msg1, msg2, msg3};
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr weather_sub_;
    rclcpp::Subscription<custom_interfaces::msg::Music>::SharedPtr music_sub_;
    feedbackList data = this->generate_feedback_list();
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Farmer>());
    rclcpp::shutdown();
    return 0;
}
