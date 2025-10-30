#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// For sleep/duration literals, required because of no rclcpp::Duration in this file
using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

// Goal Navigation Node – listens for single goals and sends them to the NavigateToPose action server
class GoalNav : public rclcpp::Node
{
public:
  GoalNav() : rclcpp::Node("goal_nav")
  {
    // Parameters for action name, expected frame, behavior tree, cancel on new goal
    action_name_ = declare_parameter<std::string>("action_name", "/navigate_to_pose");
    expected_frame_= declare_parameter<std::string>("expected_frame", "map");
    behavior_tree_ = declare_parameter<std::string>("behavior_tree", "");
    clear_on_new_ = declare_parameter<bool>("cancel_if_new_goal", false);

    // Publishers for status and distance remaining - currently unused in GUI
    status_pub_ = create_publisher<std_msgs::msg::String>("/mission/status", 10);
    dist_pub_ = create_publisher<std_msgs::msg::Float32>("/mission/feedback/distance_remaining", 10);

    // Subscriber for a single goal using qos because transient_local (to get last goal if node restarts)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/mission/goal", qos, std::bind(&GoalNav::on_goal, this, std::placeholders::_1));

    // Action client for NavigateToPos
    client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
    RCLCPP_INFO(get_logger(), "Waiting for action server: %s", action_name_.c_str());
    // Wait for the action server to be available
    while (rclcpp::ok() && !client_->wait_for_action_server(1s)) {
      RCLCPP_INFO(get_logger(), "...still waiting");
    }
    if (!rclcpp::ok()) return;
    publish_status("NO_GOAL");
    RCLCPP_INFO(get_logger(), "Ready. Subscribed to /mission/goal (expected frame: '%s').", expected_frame_.c_str());
  }

private:
  // Publish status string to /mission/status, this will be good for GUI - may need to change the topic name
  void publish_status(const std::string &s) {
    std_msgs::msg::String msg; msg.data = s; status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "STATUS -> %s", s.c_str());
  }

  // Logic to handle incoming single goal messages
  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) return; // null check
    // Check frame_id matches expected frame, this is requireed because Nav2 is sensitive to frame_id
    if (msg->header.frame_id != expected_frame_) {
      RCLCPP_ERROR(get_logger(), "Rejected goal: frame_id='%s' != expected_frame='%s'", msg->header.frame_id.c_str(), expected_frame_.c_str());
      return;
    }
    
    // If already running a goal then either ignore
    if (running_ && !clear_on_new_) {
      RCLCPP_WARN(get_logger(), "Already commuting; ignoring new goal (set cancel_if_new_goal:=true to override).");
      return;
    }

    // Cancel current goal if requested and running
    if (clear_on_new_ && running_ && current_handle_) {
      RCLCPP_WARN(get_logger(), "New goal received: cancel current and start new.");
      (void)client_->async_cancel_goal(current_handle_);
      running_ = false;
      current_handle_.reset();
    }

    // Send goal to action server (and nav2 behavior tree)
    NavigateToPose::Goal goal;
    goal.pose = *msg;
    goal.behavior_tree = behavior_tree_;

    // Set up goal options and callbacks
    auto opts = typename rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // Callbacks
    opts.goal_response_callback = [this](auto handle){
      if (!handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal REJECTED by server.");
        publish_status("NO_GOAL");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal ACCEPTED.");
        current_handle_ = handle;
        running_ = true;
        publish_status("COMMUTING");
      }
    };

    // Feedback callback to publish distance remaining
    opts.feedback_callback = [this](auto, const std::shared_ptr<const NavigateToPose::Feedback> fb){
      if (fb) {
        std_msgs::msg::Float32 d; d.data = static_cast<float>(fb->distance_remaining);
        dist_pub_->publish(d);
      }
    };

    // Result callback to handle goal completion and update status
    opts.result_callback = [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & res){
      switch (res.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Goal SUCCEEDED.");
          publish_status("AT_GOAL");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(this->get_logger(), "Goal ABORTED.");
          publish_status("ABORTED");
          break;
        case rclcpp_action::ResultCode::CANCELLED:
          RCLCPP_WARN(this->get_logger(), "Goal CANCELLED.");
          publish_status("CANCELLED");
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Goal ended with unknown result.");
          publish_status("NO_GOAL");
          break;
      }
      running_ = false;
      current_handle_.reset();
    };

    RCLCPP_INFO(get_logger(), "Sending goal (x=%.2f, y=%.2f)...", msg->pose.position.x, msg->pose.position.y);
    (void)client_->async_send_goal(goal, opts); // send goal
  }

  // Members
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr current_handle_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;

  std::string action_name_, expected_frame_, behavior_tree_;
  bool clear_on_new_{true};
  bool running_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalNav>());
  rclcpp::shutdown();
  return 0;
}
