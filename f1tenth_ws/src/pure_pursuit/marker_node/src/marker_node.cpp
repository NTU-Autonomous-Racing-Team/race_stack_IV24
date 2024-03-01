#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MarkerPublisher : public rclcpp::Node
{
  public:
    MarkerPublisher()
    : Node("marker_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MarkerPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto marker = visualization_msgs::msg::Marker();

      marker.header.frame_id = "ego_racecar/base_link";
      marker.ns = "test_namespace";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = 1;
      marker.pose.position.y = 1;
      marker.pose.position.z = 1;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      publisher_->publish(marker);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}