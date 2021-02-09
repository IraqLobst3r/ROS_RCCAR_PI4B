#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/music.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using MusicList = std::array<custom_interfaces::msg::Music, 2>;
using namespace std::chrono_literals;
using std::placeholders::_1;

class Dj : public rclcpp::Node {
   public:
    Dj() : Node("Dj") {
        publisher_ =
            this->create_publisher<custom_interfaces::msg::Music>("music", 10);
        timer_ = this->create_wall_timer(500ms,
                                         std::bind(&Dj::timer_callback, this));

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "feedback", 10, std::bind(&Dj::feedback_callback, this, _1));
    }

   private:
    void timer_callback() {
        std::random_shuffle(data.begin(), data.end());
        RCLCPP_INFO(this->get_logger(), "Title: '%s' Album: '%s' Artist: '%s'",
                    this->data[0].title.c_str(), this->data[0].album.c_str(),
                    this->data[0].artist.c_str());
        publisher_->publish(data[0]);
    }

    void feedback_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Feedback: '%s'", msg->data.c_str());
    }

    MusicList generate_music_list() {
        custom_interfaces::msg::Music music1, music2;
        music1.title = "A Sky Full of Stars";
        music1.album = "Ghost Stories";
        music1.artist = "Coldplay";

        music2.title = "Highway to Hell";
        music2.album = "Highway to Hell";
        music2.artist = "AC/DC";

        return {music1, music2};
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_interfaces::msg::Music>::SharedPtr publisher_;
    MusicList data = this->generate_music_list();
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Dj>());
    rclcpp::shutdown();
    return 0;
}
