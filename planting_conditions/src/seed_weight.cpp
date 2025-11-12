#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

#include <algorithm>
#include <limits>
#include <cmath>
#include <string>

class SeedWeightSim : public rclcpp::Node {
public:
  SeedWeightSim() : Node("seed_weight_sim")
  {
   
    hopper_capacity_g_     = declare_parameter<double>("hopper_capacity_g", 300.0);
    level_percent_         = declare_parameter<double>("initial_level_percent", 100.0);
    leak_gps_              = declare_parameter<double>("leak_gps", 0.0);
    low_thresh_            = declare_parameter<double>("low_threshold_percent", 15.0);
    stop_when_empty_       = declare_parameter<bool>("stop_when_empty", true);
    publish_step_percent_  = declare_parameter<double>("publish_step_percent", 5.0);

    last_low_state_ = (level_percent_ <= low_thresh_);

    fsm_status_pub_ = create_publisher<std_msgs::msg::String>("/mission/fsm_status", 10);

    
    percent_pub_ = create_publisher<std_msgs::msg::Int32>("/seed_hopper/level_percent", 10);
    weight_pub_  = create_publisher<std_msgs::msg::Int32>("/seed_hopper/weight_g", 10);
    low_pub_     = create_publisher<std_msgs::msg::Bool>("/seed_hopper/low", 10);

    
    flow_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/seed_hopper/outflow_gps", 10,
      [this](std_msgs::msg::Int32::SharedPtr m){ outflow_gps_cmd_ = m->data; });

  
    refill_srv_ = create_service<std_srvs::srv::Empty>(
      "/seed_hopper/refill",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>) {
        level_percent_   = 100.0;
        was_empty_       = false;
        last_pub_level_  = std::numeric_limits<double>::quiet_NaN(); 
        last_low_state_  = (level_percent_ <= low_thresh_);
        RCLCPP_INFO(this->get_logger(), "Hopper refilled to 100%%");
        publish_status("FSM: HOPPER REFILLED");
      });

    last_ = now();
    timer_ = create_wall_timer(std::chrono::milliseconds(500),
                               std::bind(&SeedWeightSim::tick, this));
  }

private:
  void publish_status(const std::string& s)
  {
    std_msgs::msg::String m;
    m.data = s;
    fsm_status_pub_->publish(m);
  }

  void tick() {
    auto t = now();
    double dt = (t - last_).seconds();
    last_ = t;

    int effective_gps = outflow_gps_cmd_;

    if (level_percent_ <= 20.0) {
    }
    if (level_percent_ <= 0.0 && stop_when_empty_) {
      effective_gps = 0;
      if (!was_empty_) {
        was_empty_ = true;
        RCLCPP_WARN(get_logger(),
          "Hopper empty; auto-stopping outflow. Call /seed_hopper/refill to resume.");
        publish_status("FSM: HOPPER EMPTY");
      }
    } else if (level_percent_ > 0.0 && was_empty_) {
      was_empty_ = false;
    }

    double out_gps = std::max(0, effective_gps) + leak_gps_;

    double dlevel = (out_gps * dt) / hopper_capacity_g_ * 100.0;
    level_percent_ -= dlevel;

    if (level_percent_ < 0.0)  level_percent_ = 0.0;
    if (level_percent_ > 100.) level_percent_ = 100.0;

    bool low_now = (level_percent_ <= low_thresh_);
    bool should_pub = false;
    if (std::isnan(last_pub_level_)) {
      should_pub = true;
    } else if ((last_pub_level_ - level_percent_) >= publish_step_percent_) {
      should_pub = true;
    } else if (low_now != last_low_state_) {
      should_pub = true;
    }

    if (should_pub) {
      const int lvl_i = static_cast<int>(std::lround(level_percent_));
      const int wt_i  = static_cast<int>(std::lround(hopper_capacity_g_ * level_percent_ / 100.0));

      RCLCPP_INFO(get_logger(), "INT-PUB hopper=%d%% weight=%dg (out=%d gps raw=%.3f%%)",
                  lvl_i, wt_i, outflow_gps_cmd_, level_percent_);

      std_msgs::msg::Int32 p; p.data = lvl_i; percent_pub_->publish(p);
      std_msgs::msg::Int32 w; w.data = wt_i;  weight_pub_->publish(w);

      std_msgs::msg::Bool low; low.data = low_now; low_pub_->publish(low);

      last_pub_level_ = level_percent_;
      last_low_state_ = low_now;
    }
  }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr percent_pub_, weight_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   low_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fsm_status_pub_;

  // Subscriptions / services / timers
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flow_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr      refill_srv_;
  rclcpp::TimerBase::SharedPtr                          timer_;
  rclcpp::Time                                          last_;

  // Parameters / state
  double hopper_capacity_g_, level_percent_, leak_gps_, low_thresh_;
  int    outflow_gps_cmd_{0};      
  bool   stop_when_empty_{true};   
  bool   was_empty_{false};        

  // Publish gating controls
  double publish_step_percent_{5.0};
  double last_pub_level_{std::numeric_limits<double>::quiet_NaN()};
  bool   last_low_state_{false};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SeedWeightSim>());
  rclcpp::shutdown();
  return 0;
}