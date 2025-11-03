#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PlantingSim : public rclcpp::Node {
public:
  PlantingSim() : Node("planting_sim"){
    plant_duration_s_ = declare_parameter("plant_duration_s", 5.0);
    plant_speed_mps_  = declare_parameter("plant_speed_mps", 0.15);
    seeds_per_run_    = declare_parameter("seeds_per_run", 10);
    cmd_vel_topic_    = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    done_pub_   = create_publisher<std_msgs::msg::Bool>("/plant/done", 10);
    count_pub_  = create_publisher<std_msgs::msg::Int32>("/plant/seeds_count", 10);
    strip_pub_  = create_publisher<std_msgs::msg::Float32>("/plant/strip_length_m", 10);
    cmd_pub_    = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/plant/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        if (active_) { res->success=false; res->message="Already planting"; return; }
        active_=true;
        start_t_=now();
        end_t_ = start_t_ + rclcpp::Duration::from_seconds(plant_duration_s_);
        RCLCPP_INFO(get_logger(),"Planting started (%.1fs).", plant_duration_s_);
        res->success=true; res->message="Planting started";
      });

    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]{ tick(); });
  }
private:
  void tick(){
    if(!active_) return;
    auto t = now();

    geometry_msgs::msg::Twist tw;
    tw.linear.x = plant_speed_mps_;
    tw.angular.z = 0.0;
    cmd_pub_->publish(tw);

    if(t >= end_t_){
      // stop
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      active_ = false;

      // publish seeds placed
      std_msgs::msg::Int32 n; n.data = seeds_per_run_; count_pub_->publish(n);

      // publish the exact straight distance we commanded
      float strip_len = static_cast<float>(plant_speed_mps_ * plant_duration_s_);
      std_msgs::msg::Float32 L; L.data = strip_len; strip_pub_->publish(L);
      RCLCPP_INFO(get_logger(),"Planting complete: %d seeds. Strip length ~ %.3f m.",
                  seeds_per_run_, strip_len);

      // done flag
      std_msgs::msg::Bool d; d.data=true; done_pub_->publish(d);
    }
  }

  // params
  double plant_duration_s_, plant_speed_mps_;
  int    seeds_per_run_;
  std::string cmd_vel_topic_;
  // state
  bool active_{false};
  rclcpp::Time start_t_, end_t_;
  // ros
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr     done_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr    count_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr  strip_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr    start_srv_;
  rclcpp::TimerBase::SharedPtr                          timer_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PlantingSim>());
  rclcpp::shutdown();
  return 0;
}
