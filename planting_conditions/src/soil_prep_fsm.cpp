#include <algorithm>
#include <cmath>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"



enum class State { IDLE, PROBING, EVALUATE, RAKING, DONE };

class SoilPrepFSM : public rclcpp::Node {
public:
  SoilPrepFSM() : rclcpp::Node("soil_prep_fsm") {
    // ---------------- Parameters ----------------
    raw_accept_thresh_ = declare_parameter<int>("raw_accept_thresh", 400);
    moisture_topic_    = declare_parameter<std::string>("moisture_topic", "/soil/moisture_raw");

    rake_speed_      = declare_parameter<double>("rake_speed_mps", 0.2);
    rake_passes_     = declare_parameter<int>("rake_passes", 3);            // forward+back = 1 pass
    rake_seg_time_s_ = declare_parameter<double>("rake_segment_time_s", 2.0);
    cmd_vel_topic_   = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    // Sunlight gate
    sunlight_topic_      = declare_parameter<std::string>("sunlight_topic", "/sunlight/ok");
    sunlight_required_   = declare_parameter<bool>("sunlight_required", true);
    sunlight_max_age_s_  = declare_parameter<double>("sunlight_max_age_s", 3.0);

    // Wait durations (now configurable)
    sunlight_wait_time_s_ = declare_parameter<double>("sunlight_wait_time_s", 10.0);
    moisture_wait_time_s_ = declare_parameter<double>("moisture_wait_time_s", 10.0);

    // Freshness for moisture (existing)
    max_age_s_ = declare_parameter<double>("moisture_max_age_s", max_age_s_);

    // ---------------- Subscriptions ----------------
    moisture_sub_ = create_subscription<std_msgs::msg::Int32>(
      moisture_topic_, 10,
      [this](const std_msgs::msg::Int32::SharedPtr m){
        last_moisture_raw_   = m->data;
        last_moisture_stamp_ = now();
        have_moisture_       = true;
      });

    sunlight_sub_ = create_subscription<std_msgs::msg::Bool>(
      sunlight_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr b){
        last_sunlight_ok_    = b->data;
        last_sunlight_stamp_ = now();
        have_sunlight_       = true;
      });

    // to show raking status
    fsm_status_pub_ = create_publisher<std_msgs::msg::String>("/mission/fsm_status", 10);

    // ---------------- Publishers ----------------
    cmd_pub_       = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    ready_pub_     = create_publisher<std_msgs::msg::Bool>("/soil_prep/ready_to_plant", 10);
    reject_pub_    = create_publisher<std_msgs::msg::Bool>("/soil_prep/rejected", 10);
    sun_block_pub_ = create_publisher<std_msgs::msg::Bool>("/soil_prep/sunlight_blocked", 10);

    // ---------------- Services ----------------
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/soil_prep/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        if (state_ != State::IDLE && state_ != State::DONE) {
          res->success = false; res->message = "Already running"; return;
        }
        stopMotion();
        reset_gates_();
        state_ = State::PROBING;
        RCLCPP_INFO(get_logger(), "Start requested");
        res->success = true; res->message = "Started";
      });

    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/soil_prep/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        stopMotion();
        reset_gates_();
        state_ = State::IDLE;
        res->success = true; res->message = "Reset";
      });

    // 10 Hz control tick
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]{ tick(); });
  }

private:
  // ---------------- Core tick ----------------
  void tick() {
    const auto t = now();

    switch (state_) {
      case State::IDLE:
        break;

      case State::PROBING: {
        // ----- Sunlight gate first -----
        if (sunlight_required_) {
          if (!have_sunlight_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for first sunlight sample on %s ...",
                                 sunlight_topic_.c_str());
            break;
          }

          if (!sunlight_probe_started_) {
            sunlight_probe_start_ = t;
            sunlight_probe_started_ = true;
            RCLCPP_INFO(get_logger(), "Waiting %.1fs before evaluating sunlight...", sunlight_wait_time_s_);
            break;
          }

          const double sun_elapsed = (t - sunlight_probe_start_).seconds();
          if (sun_elapsed < sunlight_wait_time_s_) {
            // Keep waiting for the pre-check delay to elapse
            break;
          }

          // Time to evaluate sunlight freshness + value
          const double sun_age = (t - last_sunlight_stamp_).seconds();
          if (sun_age > sunlight_max_age_s_) {
            // stale sunlight → restart the sunlight wait
            sunlight_probe_start_ = t;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "Sunlight sample is stale (age=%.2fs > %.2fs); re-waiting...",
                                 sun_age, sunlight_max_age_s_);
            break;
          }

          if (!last_sunlight_ok_) {
            // sunlight inadequate → reject and finish
            std_msgs::msg::Bool rej; rej.data = true; reject_pub_->publish(rej);
            std_msgs::msg::Bool sb;  sb.data  = true; sun_block_pub_->publish(sb);
            stopMotion();
            state_ = State::DONE;
            RCLCPP_WARN(get_logger(), "Rejected: inadequate sunlight.");
            break;
          }

          // Sunlight passed
          if (!sunlight_checked_ok_) {
            sunlight_checked_ok_ = true;
            std_msgs::msg::Bool sb; sb.data = false; sun_block_pub_->publish(sb);
            RCLCPP_INFO(get_logger(), "Sunlight OK → proceeding to moisture gate.");
          }
        } else {
          // not required; treat as passed
          if (!sunlight_checked_ok_) sunlight_checked_ok_ = true;
        }

        // ----- Moisture gate next -----
        if (!have_moisture_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                               "Waiting for first moisture sample on %s ...", moisture_topic_.c_str());
          break;
        }

        if (!moisture_probe_started_) {
          moisture_probe_start_ = t;
          moisture_probe_started_ = true;
          RCLCPP_INFO(get_logger(), "Waiting %.1fs before evaluating soil moisture...",
                      moisture_wait_time_s_);
          break;
        }

        const double moist_elapsed = (t - moisture_probe_start_).seconds();
        if (moist_elapsed < moisture_wait_time_s_) {
          // keep waiting the configured time
          break;
        }

        // Check freshness; if stale, restart moisture wait
        const double moist_age = (t - last_moisture_stamp_).seconds();
        if (moist_age > max_age_s_) {
          moisture_probe_start_ = t;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                               "Moisture sample is stale (age=%.2fs > %.2fs); re-waiting...",
                               moist_age, max_age_s_);
          break;
        }

        // Both gates satisfied → go evaluate
        state_ = State::EVALUATE;
        break;
      }

      case State::EVALUATE: {
        // Defensive re-check of sunlight (in case it flipped during moisture wait)
        if (sunlight_required_) {
          const double sun_age = (now() - last_sunlight_stamp_).seconds();
          if (sun_age > sunlight_max_age_s_ || !last_sunlight_ok_) {
            std_msgs::msg::Bool rej; rej.data = true; reject_pub_->publish(rej);
            std_msgs::msg::Bool sb;  sb.data  = true; sun_block_pub_->publish(sb);
            stopMotion();
            state_ = State::DONE;
            RCLCPP_WARN(get_logger(), "Rejected at evaluate: sunlight inadequate or stale (age=%.2fs).", sun_age);
            break;
          }
        }

        // Moisture decision
        const int r = last_moisture_raw_;
        RCLCPP_INFO(get_logger(), "Moisture RAW=%d (accept if >= %d)", r, raw_accept_thresh_);
        if (r >= raw_accept_thresh_) {
          std_msgs::msg::Bool rej; rej.data = false; reject_pub_->publish(rej);
          rake_pass_idx_ = 0;
          rake_segment_forward_ = true;
          rake_started_ = false;
          state_ = State::RAKING;
          RCLCPP_INFO(get_logger(), "Raking with %d passes, segment=%.2fs, speed=%.2f m/s",
                      rake_passes_, rake_seg_time_s_, rake_speed_);
        } else {
          std_msgs::msg::Bool rej; rej.data = true; reject_pub_->publish(rej);
          stopMotion();
          state_ = State::DONE;
          RCLCPP_WARN(get_logger(), "Rejected: soil unsuitable (RAW=%d < %d).", r, raw_accept_thresh_);
        }
        break;
      }

      case State::RAKING: {
        // need to say raking state here:
        fsm_status_pub_->publish(std_msgs::msg::String().set__data("FSM: RAKING"));
        
        if (!rake_started_) {
          rake_started_ = true;
          rake_segment_end_ = t + rclcpp::Duration::from_seconds(rake_seg_time_s_);
        }

        geometry_msgs::msg::Twist tw;
        tw.linear.x  = rake_segment_forward_ ? std::abs(rake_speed_) : -std::abs(rake_speed_);
        tw.angular.z = 0.0;
        cmd_pub_->publish(tw);

        if (t >= rake_segment_end_) {
          if (rake_segment_forward_) {
            rake_segment_forward_ = false;
            rake_segment_end_ = t + rclcpp::Duration::from_seconds(rake_seg_time_s_);
          } else {
            rake_pass_idx_++;
            if (rake_pass_idx_ >= rake_passes_) {
              stopMotion();
              state_ = State::DONE;
              std_msgs::msg::Bool ready; ready.data = true; ready_pub_->publish(ready);
              RCLCPP_INFO(get_logger(), "Raking complete: %d passes.", rake_passes_);
            } else {
              rake_segment_forward_ = true;
              rake_segment_end_ = t + rclcpp::Duration::from_seconds(rake_seg_time_s_);
            }
          }
        }
        break;
      }

      case State::DONE:
        break;
    }
  }

  void stopMotion() {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  void reset_gates_() {
    have_moisture_ = false;
    last_moisture_raw_ = 0;
    last_moisture_stamp_ = rclcpp::Time(0,0,get_clock()->get_clock_type());

    have_sunlight_ = false;
    last_sunlight_ok_ = false;
    last_sunlight_stamp_ = rclcpp::Time(0,0,get_clock()->get_clock_type());

    sunlight_probe_started_ = false;
    sunlight_checked_ok_ = false;
    moisture_probe_started_ = false;

    probe_start_time_set_ = false;
  }

  // ---------------- Parameters ----------------
  int    raw_accept_thresh_{400};
  double max_age_s_{3.0};           // freshness for moisture
  double rake_speed_{0.2};
  int    rake_passes_{3};
  double rake_seg_time_s_{2.0};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string moisture_topic_{"/soil/moisture_raw"};

  // Sunlight params
  std::string sunlight_topic_{"/sunlight/ok"};
  bool   sunlight_required_{true};
  double sunlight_max_age_s_{3.0};
  double sunlight_wait_time_s_{10.0};

  // Moisture wait param
  double moisture_wait_time_s_{10.0};

  // ---------------- State/timing ----------------
  State state_{State::IDLE};

  // Moisture
  bool have_moisture_{false};
  int  last_moisture_raw_{0};
  rclcpp::Time last_moisture_stamp_;
  bool moisture_probe_started_{false};
  rclcpp::Time moisture_probe_start_;

  // Sunlight
  bool have_sunlight_{false};
  bool last_sunlight_ok_{false};
  rclcpp::Time last_sunlight_stamp_;
  bool sunlight_probe_started_{false};
  bool sunlight_checked_ok_{false};
  rclcpp::Time sunlight_probe_start_;

  // Raking
  int  rake_pass_idx_{0};
  bool rake_segment_forward_{true};
  bool rake_started_{false};
  rclcpp::Time rake_segment_end_;

  // Legacy probe flag (not used for sunlight anymore, but kept for compatibility)
  bool probe_start_time_set_{false};

  // ---------------- ROS I/O ----------------
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  moisture_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   sunlight_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr       ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr       reject_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr       sun_block_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr      start_srv_, reset_srv_;
  rclcpp::TimerBase::SharedPtr                            timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fsm_status_pub_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SoilPrepFSM>());
  rclcpp::shutdown();
  return 0;
}