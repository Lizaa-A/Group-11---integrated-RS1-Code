#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>
#include <algorithm>

struct Pose2D {
  double x{0}, y{0}, yaw{0};
};
static inline double yaw_from_quat(const geometry_msgs::msg::Quaternion &q){
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

class IrrigationSim : public rclcpp::Node {
public:
  IrrigationSim() : Node("irrigation_sim"){
    // --- Params
    water_speed_mps_   = declare_parameter("water_speed_mps", 0.12); // forward speed during irrigation
    flow_lps_cmd_      = declare_parameter("flow_lps_cmd", 1);       // litres/sec while watering
    stop_on_low_       = declare_parameter("stop_on_low", true);
    auto_refill_       = declare_parameter("auto_refill_on_low", false);
    cmd_vel_topic_     = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_topic_        = declare_parameter<std::string>("odom_topic", "/odom");
    fallback_time_s_   = declare_parameter("fallback_time_s", 6.0);   // if no strip length/odom, time fallback

    // --- Subscriptions
    // low tank flag
    low_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/water_tank/low", 10, [this](std_msgs::msg::Bool::SharedPtr m){ low_ = m->data; });

    // strip length to cover (same value /plant publishes)
    strip_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/plant/strip_length_m", 10,
      [this](std_msgs::msg::Float32::SharedPtr m){
        last_strip_len_m_ = std::max(0.0f, m->data);
        have_strip_len_ = true;
        RCLCPP_INFO(get_logger(),"Irrigation strip length received: %.3f m", last_strip_len_m_);
      });

    // odometry
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 50,
      [this](nav_msgs::msg::Odometry::SharedPtr o){
        last_odom_ = *o; have_odom_ = true;
      });

    // --- Publishers
    flow_pub_ = create_publisher<std_msgs::msg::Int32>("/water_tank/flow_lps", 10);
    done_pub_ = create_publisher<std_msgs::msg::Bool>("/irrigate/done", 10);
    cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    // --- Optional refill client
    refill_cli_ = create_client<std_srvs::srv::Empty>("/water_tank/refill");

    // --- Start service
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/irrigate/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        if(active_){ res->success=false; res->message="Already irrigating"; return; }

        // Decide distance vs time fallback
        if (have_strip_len_ && last_strip_len_m_ > 0.0f && have_odom_) {
          target_m_ = static_cast<double>(last_strip_len_m_);
          using_time_fallback_ = false;
          start_pose_ = odomToPose2D(last_odom_);
          traveled_m_ = 0.0;
          RCLCPP_INFO(get_logger(),"Irrigation distance mode: target=%.3f m.", target_m_);
        } else {
          using_time_fallback_ = true;
          t_end_ = now() + rclcpp::Duration::from_seconds(fallback_time_s_);
          RCLCPP_WARN(get_logger(),
            "Irrigation fallback to time: %.2fs (need strip_len and odom for distance mode).",
            fallback_time_s_);
        }

        active_ = true;
        res->success = true; res->message = "Irrigation started";
      });

    // --- Tick
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]{ tick(); });
  }

private:
  static Pose2D odomToPose2D(const nav_msgs::msg::Odometry &o){
    Pose2D p; p.x = o.pose.pose.position.x; p.y = o.pose.pose.position.y;
    p.yaw = yaw_from_quat(o.pose.pose.orientation); return p;
  }

  void set_flow(int lps){
    std_msgs::msg::Int32 m; m.data = lps; flow_pub_->publish(m);
  }
  void drive_forward(){
    geometry_msgs::msg::Twist tw; tw.linear.x = std::abs(water_speed_mps_); tw.angular.z = 0.0;
    cmd_pub_->publish(tw);
  }
  void stop_motion(){
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }
  void maybe_refill(){
    if(!auto_refill_ || !refill_cli_->wait_for_service(std::chrono::milliseconds(10))) return;
    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    (void)refill_cli_->async_send_request(req);
    RCLCPP_WARN(get_logger(),"Auto-refill requested.");
  }
  void finish(bool ok){
    set_flow(0);
    stop_motion();
    active_ = false;
    std_msgs::msg::Bool d; d.data = ok; done_pub_->publish(d);
    RCLCPP_INFO(get_logger(), ok ? "Irrigation complete." : "Irrigation aborted.");
  }

  void tick(){
    if(!active_) return;

    // If we are paused due to low tank, wait here until it is refilled
    if (paused_low_) {
      stop_motion();
      set_flow(0);
      if (!low_) {
        // resume
        if (using_time_fallback_) {
          // extend deadline by the pause duration
          t_end_ = t_end_ + (now() - pause_start_);
        }
        paused_low_ = false;
        RCLCPP_INFO(get_logger(),"Resuming irrigation after refill.");
      } else {
        maybe_refill();   // optional: try auto-refill if enabled
        return;           // stay paused
      }
    }

    if (stop_on_low_ && low_) {
      RCLCPP_WARN(get_logger(),"Low tank during irrigation to pausing.");
      paused_low_ = true;
      pause_start_ = now();
      stop_motion();
      set_flow(0);
      maybe_refill();     // optional auto-refill
      return;
    }

    // command forward motion and flow
    drive_forward();
    set_flow(flow_lps_cmd_);

    if (!using_time_fallback_) {
      if (have_odom_) {
        Pose2D cur = odomToPose2D(last_odom_);
        traveled_m_ = std::hypot(cur.x - start_pose_.x, cur.y - start_pose_.y);
        if (traveled_m_ >= target_m_) {
          RCLCPP_INFO(get_logger(),"Irrigated distance %.3f / %.3f m.", traveled_m_, target_m_);
          finish(true);
        }
      } else {
        // lost odom mid-run → stop safely
        RCLCPP_WARN(get_logger(),"Lost odometry during irrigation; stopping.");
        finish(false);
      }
    } else {
      if (now() >= t_end_) {
        finish(true);
      }
    }
  }

  // Params
  double water_speed_mps_{0.12};
  int    flow_lps_cmd_{1};
  bool   stop_on_low_{true};
  bool   auto_refill_{false};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string odom_topic_{"/odom"};
  double fallback_time_s_{6.0};

  // State
  bool active_{false};
  bool low_{false};
  bool paused_low_{false};
  rclcpp::Time pause_start_;


  // Distance-mode state
  bool   using_time_fallback_{true};
  bool   have_strip_len_{false};
  bool   have_odom_{false};
  float  last_strip_len_m_{0.0f};
  nav_msgs::msg::Odometry last_odom_;
  Pose2D start_pose_;
  double traveled_m_{0.0};
  double target_m_{0.0};
  rclcpp::Time t_end_;

  // ROS I/O
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr     low_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr  strip_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr       flow_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        done_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr       start_srv_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr          refill_cli_;
  rclcpp::TimerBase::SharedPtr                             timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IrrigationSim>());
  rclcpp::shutdown();
  return 0;
}