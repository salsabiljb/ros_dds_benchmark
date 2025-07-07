// src/image_subscriber_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <unordered_map>

using namespace std::chrono_literals;

class MultiImageSubscriber : public rclcpp::Node
{
public:
  MultiImageSubscriber()
  : Node("multi_image_subscriber")
  {
    std::filesystem::create_directories("logs");

    /* Wait a few seconds to see topics */
    timer_ = this->create_wall_timer(
      5s,
      std::bind(&MultiImageSubscriber::discover_topics, this));
  }

private:
  /* print img topics and create subs */
  void discover_topics()
  {
    timer_->cancel();   // run once

    auto topics_and_types = this->get_topic_names_and_types();
    RCLCPP_INFO(this->get_logger(), "Scanning for image topics…");

    for (const auto & pair : topics_and_types)
    {
      const std::string & topic  = pair.first;
      const auto &        types  = pair.second;

      if (std::find(types.begin(), types.end(),
                    "sensor_msgs/msg/Image") != types.end())
      {
        std::string clean = topic;
        std::replace(clean.begin(), clean.end(), '/', '_');

        /* open CSV for each related topic */
        auto & log = logs_[topic] =
          std::ofstream("logs/" + clean + ".csv");
        log << "seq,frame_id,img_timestamp,recv_timestamp,latency\n";

        /* create subscription */
        subs_.push_back(
          this->create_subscription<sensor_msgs::msg::Image>(
            topic,
            rclcpp::SensorDataQoS(),
            [this, topic](sensor_msgs::msg::Image::SharedPtr msg)
            {
              static std::unordered_map<std::string, uint64_t> seq;
              auto now      = this->get_clock()->now();
              rclcpp::Time ts(msg->header.stamp);
              auto latency  = now - ts;
              
              auto & log = logs_[topic];
              log << seq[topic]++ << ','
                  << msg->header.frame_id << ','
                  << ts.nanoseconds()   << ','
                  << now.nanoseconds()        << ','
                  << latency.nanoseconds()    << '\n';
            }));

        RCLCPP_INFO(this->get_logger(),
                    "Subscribed to %s (csv: logs/%s.csv)",
                    topic.c_str(), clean.c_str());
      }
    }

    if (subs_.empty())
      RCLCPP_WARN(this->get_logger(),
                  "No sensor_msgs/Image topics found!");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;
  std::unordered_map<std::string, std::ofstream> logs_;
};

/* main */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}

