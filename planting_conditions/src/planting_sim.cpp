#include <chrono>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PlantingSim : public rclcpp::Node {
public:
  PlantingSim() : Node("planting_sim"){
    // ------------ Core planting params ------------
    plant_duration_s_ = declare_parameter("plant_duration_s", 5.0);
    plant_speed_mps_  = declare_parameter("plant_speed_mps", 0.15);
    seeds_per_run_    = declare_parameter("seeds_per_run", 10);
    cmd_vel_topic_    = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    // ------------ Seed outflow (for hopper weight) ------------
    // If >0, use explicit seeds/sec; otherwise derive from seeds_per_run / plant_duration_s
    seeds_per_second_ = declare_parameter("seeds_per_second", -1.0);
    grams_per_seed_   = declare_parameter("grams_per_seed", 10);   // 40 mg per seed by default
    stop_on_low_      = declare_parameter("stop_on_low", true);       // pause if hopper low
    outflow_topic_    = declare_parameter<std::string>("outflow_topic", "/seed_hopper/outflow_gps");

    // ------------ IO ------------
    done_pub_   = create_publisher<std_msgs::msg::Bool>    ("/plant/done", 10);
    count_pub_  = create_publisher<std_msgs::msg::Int32>   ("/plant/seeds_count", 10);
    strip_pub_  = create_publisher<std_msgs::msg::Float32> ("/plant/strip_length_m", 10);
    cmd_pub_    = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    outflow_pub_= create_publisher<std_msgs::msg::Int32>   (outflow_topic_, 10);

    // optional hopper low pause
    low_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/seed_hopper/low", 10,
      [this](std_msgs::msg::Bool::SharedPtr m){ low_ = m->data; });

    // /plant/start service
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/plant/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        if (active_) { res->success=false; res->message="Already planting"; return; }
        active_ = true; paused_low_ = false;
        start_t_ = now();
        end_t_   = start_t_ + rclcpp::Duration::from_seconds(plant_duration_s_);

        // compute placement rate + outflow gps
        if (seeds_per_second_ > 0.0) {
          seeds_rate_ = seeds_per_second_;
        } else {
          seeds_rate_ = (plant_duration_s_ > 0.0) ? (static_cast<double>(seeds_per_run_) / plant_duration_s_) : 0.0;
        }
        outflow_gps_cmd_ = std::max(0, static_cast<int>(std::lround(seeds_rate_ * grams_per_seed_)));

        RCLCPP_INFO(get_logger(),
          "Planting started (T=%.1fs). rate=%.3f seeds/s, outflow=%d gps",
          plant_duration_s_, seeds_rate_, outflow_gps_cmd_);

        res->success=true; res->message="Planting started";
      });

    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]{ tick(); });
  }

private:
  void set_outflow(int gps){
    std_msgs::msg::Int32 m; m.data = gps; outflow_pub_->publish(m);
  }
  void stop_motion(){
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  void tick(){
    if(!active_) return;

    // Pause on low hopper if requested
    if (stop_on_low_ && low_) {
      if (!paused_low_) {
        RCLCPP_WARN(get_logger(),"Hopper low; pausing seeding outflow.");
        paused_low_ = true;
      }
      stop_motion();
      set_outflow(0);
      return; // remain paused until /seed_hopper/low becomes false
    } else if (paused_low_ && !low_) {
      RCLCPP_INFO(get_logger(),"Resuming seeding after refill.");
      paused_low_ = false;
    }

    // Drive forward
    geometry_msgs::msg::Twist tw;
    tw.linear.x = plant_speed_mps_;
    tw.angular.z = 0.0;
    cmd_pub_->publish(tw);

    // Command seed outflow (grams/sec) so hopper weight integrates down
    set_outflow(outflow_gps_cmd_);

    // Finish by time
    if (now() >= end_t_){
      // stop motion + stop seed flow
      stop_motion();
      set_outflow(0);
      active_ = false;

      // publish seeds placed
      std_msgs::msg::Int32 n; n.data = seeds_per_run_; count_pub_->publish(n);

      // publish commanded straight distance
      float strip_len = static_cast<float>(plant_speed_mps_ * plant_duration_s_);
      std_msgs::msg::Float32 L; L.data = strip_len; strip_pub_->publish(L);
      RCLCPP_INFO(get_logger(),"Planting complete: %d seeds. Strip length ~ %.3f m.",
                  seeds_per_run_, strip_len);

      // done flag
      std_msgs::msg::Bool d; d.data=true; done_pub_->publish(d);
    }
  }

  // params
  double plant_duration_s_{}, plant_speed_mps_{};
  int    seeds_per_run_{};
  std::string cmd_vel_topic_;
  // outflow/seed params
  double seeds_per_second_{-1.0};
  double grams_per_seed_{0.040};
  bool   stop_on_low_{true};
  std::string outflow_topic_;
  // state
  bool active_{false};
  bool low_{false};
  bool paused_low_{false};
  double seeds_rate_{0.0};
  int    outflow_gps_cmd_{0};
  rclcpp::Time start_t_, end_t_;
  // ros
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        done_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr       count_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr     strip_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr       outflow_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr     low_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr       start_srv_;
  rclcpp::TimerBase::SharedPtr                             timer_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PlantingSim>());
  rclcpp::shutdown();
  return 0;
}