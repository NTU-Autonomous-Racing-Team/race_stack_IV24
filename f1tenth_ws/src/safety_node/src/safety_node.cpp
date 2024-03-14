#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/AckermannDriveStamped.hpp"
using std::placeholders::_1;

class SafetyNode: public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("safety_node")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "safety/drive", 10, std::bind(&MinimalSubscriber::drive_callback, this, _1));
    }

  private:
    void drive_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
           
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}