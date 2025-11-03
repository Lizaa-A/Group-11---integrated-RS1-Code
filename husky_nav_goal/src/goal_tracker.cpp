#include <deque>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

// Goal Tracker Node – manages a queue of goals and sends them one at a time to /mission/goal.
// It waits for permission via /mission/permit_next before sending each goal if require_permission is true.

class GoalTracker : public rclcpp::Node
{
public:
  GoalTracker() : rclcpp::Node("goal_tracker")
  {
    // Parameters for expected frame and permission requirement
    expected_frame_ = declare_parameter<std::string>("expected_frame", "map");
    require_permission_ = declare_parameter<bool>("require_permission", true);
    
    // Publishers
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mission/goal", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/mission/waypoints_viz", 10);

    // Subscriptions with qos because transient_local for path (to get last published path if node restarts)
    // auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Inputs for path and status
    path_sub_ = create_subscription<nav_msgs::msg::Path>("/mission/waypoints", qos, std::bind(&GoalTracker::on_path, this, std::placeholders::_1));
    status_sub_ = create_subscription<std_msgs::msg::String>("/mission/status", qos, std::bind(&GoalTracker::on_status, this, std::placeholders::_1));

    // Permit gate for sending next goal
    // THIS MAY BE ISSUE: auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    permit_sub_ = create_subscription<std_msgs::msg::Empty>("/mission/permit_next", 10, [this](std_msgs::msg::Empty::SharedPtr)
    {
      permitted_ = true; // allow sending next goal
      RCLCPP_INFO(this->get_logger(), "[permit_next] Permit received, allowed to send next goal.");
      // If we’re idle and not waiting on a current goal, try to send immediately
      if (!waiting_ && (last_status_.empty() || last_status_=="NO_GOAL" || last_status_=="AT_GOAL")) {
        send_next_if_any(); // try to send next goal
      }
    });
    RCLCPP_INFO(get_logger(), "GoalTracker ready. expected_frame='%s', require_permission=%s", expected_frame_.c_str(), require_permission_ ? "true" : "false");
  }

private:
  // Logic to handle incoming Path messages/goals
  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg) return; // null check
    // Validate incoming path
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received Path with 0 poses; ignoring.");
      return;
    }
    // Check frame_id matches expected frame
    if (msg->header.frame_id != expected_frame_) {
      RCLCPP_ERROR(get_logger(), "Path frame_id='%s' != expected_frame='%s' — ignoring.", msg->header.frame_id.c_str(), expected_frame_.c_str());
      return;
    }

    // Replace queue with new goals
    queue_.clear();
    for (const auto &p : msg->poses) {
      if (p.header.frame_id != expected_frame_) {
        RCLCPP_WARN(get_logger(), "Pose with frame_id='%s' not '%s' — skipping.", p.header.frame_id.c_str(), expected_frame_.c_str());
        continue;
      }
      queue_.push_back(p);
    }

    // Re-publish for markers
    last_path_ = *msg;
    path_pub_->publish(last_path_);

    // Log queue size
    RCLCPP_INFO(get_logger(), "Qued %zu goals.", queue_.size());

    // If robot is idle, send first  (only if permission allows)
    if (!waiting_ && (last_status_.empty() || last_status_=="NO_GOAL" || last_status_=="AT_GOAL")) {
     if (!require_permission_ || permitted_) {
        send_next_if_any();
      } else { // waiting for permission
        RCLCPP_INFO(get_logger(), "Waiting for /mission/permit_next before sending first goal.");
      }
    }
  }

  // Logic to handle incoming status messages
  void on_status(const std_msgs::msg::String::SharedPtr msg)
  {
    last_status_ = msg->data;
    // Handle status by saying whether we are waiting or can send next
    if (last_status_ == "COMMUTING") {
      waiting_ = true; // we have a goal in flight
    } else if (last_status_ == "NO_GOAL" || last_status_ == "AT_GOAL") {
      waiting_ = false;
      if (!require_permission_) {
        send_next_if_any();
      }
    } else if (last_status_ == "COMMUTING") {
      waiting_ = true;
    } else {
      RCLCPP_INFO(get_logger(), "At goal / idle. Waiting for /mission/permit_next.");
    }
    // If COMMUTING / ABORTED / CANCELED we do nothing immediately.
  }

  // Send next goal in queue if any
  void send_next_if_any()
  {
    if (waiting_) return; // already sent one, wait for status
    if (queue_.empty()) {
      RCLCPP_INFO(get_logger(), "No pending goals in queue.");
      return;
    }
    
    // Respect permission gate
    if (require_permission_ && !permitted_ && first_goal_sent_) {
      RCLCPP_INFO(get_logger(), "Permission required and not granted yet; not sending.");
      return;
    }

    // Pop and send next goal
    auto next = queue_.front();
    queue_.pop_front();

    // Ensure header.frame_id
    if (next.header.frame_id.empty()) next.header.frame_id = expected_frame_;

    goal_pub_->publish(next);
    waiting_ = true;
    // consume one permit
    if (require_permission_) {
      permitted_ = false;
    }
    RCLCPP_INFO(get_logger(), "Published next goal (x=%.2f, y=%.2f). %zu remaining.",next.pose.position.x, next.pose.position.y, queue_.size());
  }
  // Members
  bool waiting_{false};
  bool require_permission_{true};
  bool permitted_{true};
  bool first_goal_sent_{false};

  std::string expected_frame_{"map"};
  std::string last_status_;
  std::deque<geometry_msgs::msg::PoseStamped> queue_;
  nav_msgs::msg::Path last_path_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr permit_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

// Main function temporarily here
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalTracker>());
  rclcpp::shutdown();
  return 0;
}
