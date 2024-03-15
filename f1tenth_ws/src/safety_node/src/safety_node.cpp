#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int.hpp"
#include "ackermann_msgs/msg/AckermannDriveStamped.hpp"
using std::placeholders::_1;

class SafetyNode: public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("safety_node"), drive_gain(0)
    {
      /* Pass through */
      drive_subscriber = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "safety/drive", 1, std::bind(&MinimalSubscriber::drive_callback, this, _1));

      drive_publisher = this->create_publisher<ackermann_msgs::msg::String>("drive", 1);

      /* Triggers for braking */
      ttc_subscriber = this->create_subscription<std_msgs::msg::Int>(
      "ttc", 10, std::bind(&MinimalSubscriber::ttc_callback, this, _1));

      teleop_subscriber = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "teleop", 10, std::bind(&MinimalSubscriber::dead_man_callback, this, _1));
    }

  private:
    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) const
    {
      auto message = msg;
      msg->drive->speed = msg->drive->speed * drive_gain;
      message.drive.steering_angle = message.drive.steering_angle * drive_gain;
      drive_publisher->publish(message);
    }

    void ttc_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) const
    {
      if (msg.data < 0.5)
      {
        drive_gain = 0;
      }
      else
      {
        drive_gain = 1;
      }
    }

    void dead_man_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) const
    {
      msg.header.stamp = this->now();

    }

    rclcpp::Subscription<ackermann_msgs_msgs::msg::AckermannDriveStamped>::SharedPtr drive_subscriber;
    rclcpp::Publisher<ackermann_msgs_msgs::msg::AckermannDriveStamped::SharedPtr drive_publisher;
    rclcpp::Subscription<std_msgs::msg::Int>::SharedPtr ttc_subscriber;
    rclcpp::Subscription<ackermann_msgs_msgs::msg::AckermannDriveStamped>::SharedPtr dead_man_subscriber;
    size_t drive_gain;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}



#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}