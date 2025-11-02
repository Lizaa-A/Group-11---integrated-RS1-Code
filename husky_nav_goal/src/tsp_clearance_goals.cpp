#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// Simple struct for 2D points and squared distance, useful below
struct XY { double x{0.0}, y{0.0}; };
static inline double d2(const XY& a, const XY& b){ double dx=a.x-b.x, dy=a.y-b.y; return dx*dx+dy*dy; }

// TSP Clearance Goals Node – Nearest Neighbor heuristic to order the best path to go to multuple goals, using AMCL pose as seed if available
// Subscribes to unordered nav_msgs/Path, publishes ordered nav_msgs/Path. 
// Uses greedy Nearest-Neighbour (O(N^2)), fast for small N
class TspClearanceGoals : public rclcpp::Node {
public:
  TspClearanceGoals() : rclcpp::Node("tsp_clearance_goals")
  {
    in_topic_ = declare_parameter<std::string>("in_topic", "perception/clearances"); // unordered in
    out_topic_ = declare_parameter<std::string>("out_topic", "/mission/waypoints"); // ordered out
    map_frame_ = declare_parameter<std::string>("map_frame", "map"); // output frame_id
    force_map_frame_ = declare_parameter<bool>("force_map_frame", true); // enforce frame_id
    seed_from_amcl_ = declare_parameter<bool>("seed_from_amcl", true); // use AMCL pose as seed

    // AMCL subscription for seeding, because starting point affects NN result
    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", rclcpp::QoS(1), [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
      {
        // Store AMCL x,y
        start_.x = msg->pose.pose.position.x;
        start_.y = msg->pose.pose.position.y;
        have_amcl_.store(true);
      });
    // Input subscription for unordered path and output publisher for ordered path, qos because transient_local
    // transient_local so that downstream nodes (goal_tracker, goals_markers) can see last published path even if they start later
    out_pub_ = create_publisher<nav_msgs::msg::Path>(out_topic_, rclcpp::QoS(1).reliable().transient_local());
    auto in_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_ = create_subscription<nav_msgs::msg::Path>(in_topic_, in_qos, std::bind(&TspClearanceGoals::onIncomingPath, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "TSP node ready. in='%s' → out='%s'", in_topic_.c_str(), out_topic_.c_str());
  }

private:
  // Logic to handle incoming unordered Path messages/goals
  void onIncomingPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // Ignore empty goals
    if (!msg || msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Incoming Path is empty; ignoring.");
      return;
    }

    // Determine seed point for NN
    // Use AMCL pose if available, otherwise use first goal
    XY seed{}; // default first goal
    if (seed_from_amcl_ && have_amcl_.load()) {
      seed = start_;
      RCLCPP_INFO(get_logger(), "Seeding NN from AMCL: (%.2f, %.2f)", seed.x, seed.y);
    } else { // no AMCL yet
      seed.x = msg->poses.front().pose.position.x;
      seed.y = msg->poses.front().pose.position.y;
      if (seed_from_amcl_) RCLCPP_WARN(get_logger(), "No /amcl_pose yet; seeding from first goal.");
    }

    // Nearest-Neighbour TSP heuristic to order the goals algorithm
    const size_t N = msg->poses.size();
    std::vector<size_t> order; order.reserve(N);
    std::vector<bool> used(N, false);

    // Greedy NN works by repeatedly finding nearest unused goal and adding it to the path thnen moving there
    XY current = seed;
    for (size_t k = 0; k < N; ++k) {
      size_t best = N; double bestd = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < N; ++i) {
        if (used[i]) continue;
        XY g{ msg->poses[i].pose.position.x, msg->poses[i].pose.position.y };
        const double dd = d2(current, g);
        if (dd < bestd) { bestd = dd; best = i; }
      }
      used[best] = true;
      order.push_back(best);
      // Move to that goal that is the nearest compared to current
      current = XY{ msg->poses[best].pose.position.x, msg->poses[best].pose.position.y };
    }

    nav_msgs::msg::Path out;
    out.header.frame_id = map_frame_;
    out.header.stamp    = now();
    out.poses.reserve(N);
    // Fill in poses in the determined order
    for (size_t j = 0; j < N; ++j) {
      auto ps = msg->poses[ order[j] ];
      if (force_map_frame_) ps.header.frame_id = map_frame_;
      ps.header.stamp = out.header.stamp;
      out.poses.push_back(ps);
    }
    // Publish ordered path
    out_pub_->publish(out);
    RCLCPP_INFO(get_logger(), "Published ordered Path → %s (%zu poses).", out_topic_.c_str(), out.poses.size());
  }

  // Params
  std::string in_topic_, out_topic_, map_frame_;
  bool force_map_frame_{true}, seed_from_amcl_{true};

  // AMCL seed
  std::atomic<bool> have_amcl_{false};
  XY start_{};

  // ROS I/O connections
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr out_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
};

// Main function temporarily here
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TspClearanceGoals>());
  rclcpp::shutdown();
  return 0;
}