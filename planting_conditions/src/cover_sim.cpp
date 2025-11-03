#include <chrono>
#include <cmath>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CoverSim : public rclcpp::Node {
public:
  CoverSim() : Node("cover_sim_no_odom")
  {
    // Parameters
    back_speed_mps_        = declare_parameter("back_speed_mps", 0.10);         // absolute magnitude; sent as negative
    cmd_vel_topic_         = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    cover_fallback_time_s_ = declare_parameter("cover_fallback_time_s", 3.0);   // used if no strip length received
    passes_                = declare_parameter("passes", 1);                     // number of full-length traversals

    if (passes_ < 1) passes_ = 1;

    // IO
    strip_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/plant/strip_length_m", 10,
      [this](std_msgs::msg::Float32::SharedPtr m){
        last_strip_len_m_ = std::max(0.0f, m->data);
        have_strip_len_   = true;
        RCLCPP_INFO(get_logger(), "Received strip length: %.3f m", last_strip_len_m_);
      });

    cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    done_pub_ = create_publisher<std_msgs::msg::Bool>("/cover/done", 10);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/cover/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        if (active_) {
          res->success = false; res->message = "Already covering";
          return;
        }

        const double speed = std::abs(back_speed_mps_);
        if (speed < 1e-6) {
          res->success = false; res->message = "back_speed_mps too small (zero)";
          RCLCPP_ERROR(get_logger(), "Refusing to start: back_speed_mps=%.6f", back_speed_mps_);
          return;
        }

        active_ = true;
        started_time_ = now();

        // Compute total duration
        if (have_strip_len_ && last_strip_len_m_ > 0.0f) {
          const double distance = static_cast<double>(last_strip_len_m_) * static_cast<double>(passes_);
          total_duration_ = rclcpp::Duration::from_seconds(distance / speed);
          using_fallback_time_ = false;
          RCLCPP_INFO(get_logger(),
            "Covering BACKWARD by time to match %.3f m × %d passes at %.3f m/s → %.2f s total.",
            static_cast<double>(last_strip_len_m_), passes_, speed, distance / speed);
        } else {
          total_duration_ = rclcpp::Duration::from_seconds(cover_fallback_time_s_);
          using_fallback_time_ = true;
          RCLCPP_WARN(get_logger(),
            "No strip length yet; using fixed fallback time: %.2f s (passes=%d ignored).",
            cover_fallback_time_s_, passes_);
        }

        end_time_ = started_time_ + total_duration_;

        res->success = true; res->message = "Covering started";
      });

    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]{ tick(); });
  }

private:
  void tick(){
    if (!active_) return;

    auto t = now();

    // Constant backward motion
    geometry_msgs::msg::Twist tw;
    tw.linear.x  = -std::abs(back_speed_mps_); // backward
    tw.angular.z = 0.0;
    cmd_pub_->publish(tw);

    if (t >= end_time_) {
      // stop
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      active_ = false;

      std_msgs::msg::Bool d; d.data = true;
      done_pub_->publish(d);

      if (using_fallback_time_) {
        RCLCPP_INFO(get_logger(), "Covering complete (TIME fallback).");
      } else {
        RCLCPP_INFO(get_logger(), "Covering complete (TIME from strip length × passes).");
      }
    }
  }

  // Params
  double      back_speed_mps_{0.10};
  std::string cmd_vel_topic_{"/cmd_vel"};
  double      cover_fallback_time_s_{3.0};
  int         passes_{1};

  // State
  bool active_{false};
  bool using_fallback_time_{false};
  bool have_strip_len_{false};
  float last_strip_len_m_{0.0f};

  rclcpp::Time started_time_;
  rclcpp::Time end_time_;
  rclcpp::Duration total_duration_{0,0};

  // ROS I/O
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr strip_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr       done_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr      start_srv_;
  rclcpp::TimerBase::SharedPtr                            timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoverSim>());
  rclcpp::shutdown();
  return 0;
}