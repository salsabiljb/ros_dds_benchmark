#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <fstream>
#include <filesystem> 

class ImageSub : public rclcpp::Node {
public:
    ImageSub() : Node("image_subscriber"), seq_(0) { //added
        std::filesystem::create_directories("logs");
        log_.open("logs/timestamps.csv");
        log_ << "seq,frame_id,img_timestamp,recv_timestamp,latency\n";

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                auto now = this->now();
                rclcpp::Time img_time(msg->header.stamp);
                auto latency = now - img_time;

                log_ << seq_++ << ","                       // sequence number
                     << msg->header.frame_id << ","
                     << img_time.seconds() << ","
                     << now.seconds() << ","
                     << latency.seconds() << "\n";
            });
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::ofstream log_;
    uint64_t seq_;
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSub>());
    rclcpp::shutdown();
    return 0;
}
