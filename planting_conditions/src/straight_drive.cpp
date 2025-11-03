#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class StraightDriver : public rclcpp::Node {
public:
  StraightDriver() : rclcpp::Node("straight_driver")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&StraightDriver::tick, this));
  }
private:
  void tick() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.5;   // forward (m/s)
    cmd.angular.z = 0.0;  // straight
    pub_->publish(cmd);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StraightDriver>());
  rclcpp::shutdown();
  return 0;
}
