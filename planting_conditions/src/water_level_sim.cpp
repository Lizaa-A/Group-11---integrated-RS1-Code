#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <limits>
#include <cmath>

class WaterLevelSim : public rclcpp::Node {
public:
  WaterLevelSim() : Node("water_level_sim")
  {
  
    tank_volume_l_        = declare_parameter<double>("tank_volume_l", 10.0);
    level_percent_        = declare_parameter<double>("initial_level_percent", 100.0);
    leak_lps_             = declare_parameter<double>("leak_lps", 0.0);
    low_thresh_           = declare_parameter<double>("low_threshold_percent", 15.0);
    stop_when_empty_      = declare_parameter<bool>("stop_when_empty", true);
    publish_step_percent_ = declare_parameter<double>("publish_step_percent", 5.0);

    last_low_state_ = (level_percent_ <= low_thresh_);

    fsm_status_pub_ = create_publisher<std_msgs::msg::String>("/mission/fsm_status", 10);

    level_pub_ = create_publisher<std_msgs::msg::Int32>("/water_tank/level_percent", 10);
    vol_pub_   = create_publisher<std_msgs::msg::Int32>("/water_tank/volume_l", 10);
    low_pub_   = create_publisher<std_msgs::msg::Bool>("/water_tank/low", 10);


    flow_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/water_tank/flow_lps", 10,
      [this](std_msgs::msg::Int32::SharedPtr m){ flow_lps_ = m->data; });

    refill_srv_ = create_service<std_srvs::srv::Empty>(
      "/water_tank/refill",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>) {
        level_percent_   = 100.0;
        was_empty_       = false;
        last_pub_level_  = std::numeric_limits<double>::quiet_NaN(); 
        last_low_state_  = (level_percent_ <= low_thresh_);
        RCLCPP_INFO(this->get_logger(), "Tank refilled to 100%%");
        publish_status("FSM: TANK REFILLED");
      });

    last_ = now();
    timer_ = create_wall_timer(std::chrono::milliseconds(500),
                               std::bind(&WaterLevelSim::tick, this));
  }

private:
  void publish_status(const std::string& s)
  {
    std_msgs::msg::String m;
    m.data = s;
    fsm_status_pub_->publish(m);
  }
  void tick() {
    // Compute elapsed time since last second
    auto t = now();
    double dt = (t - last_).seconds();
    last_ = t;

    int effective_flow = flow_lps_;
    if (level_percent_ <= 20.0) {
      publish_status("FSM: TANK EMPTY");
    }
    if (level_percent_ <= 0.0 && stop_when_empty_) {
      effective_flow = 0;
      if (!was_empty_) {
        was_empty_ = true;
        RCLCPP_WARN(get_logger(),
          "Tank empty; auto-stopping flow. Call /water_tank/refill to resume.");
      }
    } else if (level_percent_ > 0.0 && was_empty_) {
      was_empty_ = false;
    }


    double out_lps = std::max(0, effective_flow) + leak_lps_;

    // Convert outflow over dt into % drop:
    double dlevel = (out_lps * dt) / tank_volume_l_ * 100.0;
    level_percent_ -= dlevel;

    // limits [0, 100]
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

    // If publishing, send % level, litres, and low flag (true or false)
    if (should_pub) {
      const int lvl_i = static_cast<int>(std::lround(level_percent_));
      const int vol_i = static_cast<int>(std::lround(tank_volume_l_ * level_percent_ / 100.0));

      RCLCPP_INFO(get_logger(), "INT-PUB lvl=%d%% vol=%dL (flow=%d lps raw=%.3f%%)", 
                  lvl_i, vol_i, flow_lps_, level_percent_);

      std_msgs::msg::Int32 p; p.data = lvl_i; level_pub_->publish(p);
      std_msgs::msg::Int32 v; v.data = vol_i; vol_pub_->publish(v);

      std_msgs::msg::Bool low; low.data = low_now; low_pub_->publish(low);

      // Update gating state
      last_pub_level_ = level_percent_;
      last_low_state_ = low_now;
    }
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr level_pub_, vol_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   low_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flow_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr    refill_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fsm_status_pub_;

  // parameters
  double tank_volume_l_, level_percent_, leak_lps_, low_thresh_;
  int    flow_lps_{0};           // commanded outflow (lps)
  bool   stop_when_empty_{true}; // auto-stop flow at 0% if true
  bool   was_empty_{false};      // edge-trigger to warn once

  // Publish gating controls
  double publish_step_percent_{5.0}; // min drop between publishes (updates at each 10% drop)
  double last_pub_level_{std::numeric_limits<double>::quiet_NaN()};
  bool   last_low_state_{false};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaterLevelSim>());
  rclcpp::shutdown();
  return 0;
}