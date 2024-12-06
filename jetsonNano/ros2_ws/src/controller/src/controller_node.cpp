#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/range.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
  public:
    ControllerNode()
    : Node("controller_node")
    {
      m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("instructions", 10);
      m_timer = this->create_wall_timer(
      500ms, std::bind(&ControllerNode::timer_callback, this));
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;

    void timer_callback()
    {
      RCLCPP_INFO(this->get_logger(), "Publishing instructions");

      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 0;
      message.linear.y = 0;
      message.linear. z = 0;

      message.angular.x = 0;
      message.angular.y = 0;
      message.angular.z = 0;
      m_publisher->publish(message);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}