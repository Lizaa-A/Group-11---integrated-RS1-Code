#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

struct XY { double x{0.0}, y{0.0}; };
static inline double d2(const XY& a, const XY& b){ double dx=a.x-b.mx, dy=a.y-b.y; return dx*dx+dy*dy; }

class TspClearanceGoals : public rclcpp::Node {
public:
  TspClearanceGoals()
  : rclcpp::Node("tsp_clearance_goals")
  {
    // Parameters
    in_topic_  = declare_parameter<std::string>("in_topic",  "ugv/clearance_goals_unordered");
    out_topic_ = declare_parameter<std::string>("out_topic", "/mission/waypoints"); // <- feed waypoint_navigator
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    force_map_frame_ = declare_parameter<bool>("force_map_frame", true);
    seed_from_amcl_  = declare_parameter<bool>("seed_from_amcl", true);

    // Optional AMCL seed (for a better start for NN heuristic)
    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", rclcpp::QoS(1),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
        start_.x = msg->pose.pose.position.x;
        start_.y = msg->pose.pose.position.y;
        have_amcl_.store(true);
      });

    // Publisher: ordered Path to waypoint_navigator
    // Transient local so late joiners (your waypoint_navigator) can get the last mission
    out_pub_ = create_publisher<nav_msgs::msg::Path>(
      out_topic_, rclcpp::QoS(1).reliable().transient_local());

    // Subscriber: incoming (already filtered) waypoints
    // Transient local lets us receive the last published path even if it was sent slightly before we came up
    auto in_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_ = create_subscription<nav_msgs::msg::Path>(
      in_topic_, in_qos,
      std::bind(&TspClearanceGoals::onIncomingPath, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "TSP node ready. in_topic='%s', out_topic='%s'",
                in_topic_.c_str(), out_topic_.c_str());
  }

private:
  void onIncomingPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg || msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Incoming Path is empty; ignoring.");
      return;
    }

    // Determine NN seed
    XY seed{};
    if (seed_from_amcl_ && have_amcl_.load()) {
      seed = start_;
      RCLCPP_INFO(get_logger(), "Seeding NN from AMCL: (%.2f, %.2f)", seed.x, seed.y);
    } else {
      seed.x = msg->poses.front().pose.position.x;
      seed.y = msg->poses.front().pose.position.y;
      if (seed_from_amcl_)
        RCLCPP_WARN(get_logger(), "No /amcl_pose yet; seeding NN from first goal.");
    }

    // Indices of incoming poses
    const size_t N = msg->poses.size();
    std::vector<size_t> order; order.reserve(N);
    std::vector<bool> used(N, false);

    XY cur = seed;
    for (size_t k = 0; k < N; ++k) {
      size_t best = N; double bestd = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < N; ++i) {
        if (used[i]) continue;
        XY g{ msg->poses[i].pose.position.x, msg->poses[i].pose.position.y };
        const double dd = d2(cur, g);
        if (dd < bestd) { bestd = dd; best = i; }
      }
      used[best] = true;
      order.push_back(best);
      cur = XY{ msg->poses[best].pose.position.x, msg->poses[best].pose.position.y };
    }

    // Build ordered Path
    nav_msgs::msg::Path out;
    out.header.frame_id = map_frame_;
    out.header.stamp    = now();
    out.poses.reserve(N);
    for (size_t j = 0; j < N; ++j) {
      auto ps = msg->poses[ order[j] ];
      if (force_map_frame_) ps.header.frame_id = map_frame_;
      ps.header.stamp = out.header.stamp;
      out.poses.push_back(ps);
    }

    out_pub_->publish(out);
    RCLCPP_INFO(get_logger(), "Published ordered Path → %s (%zu poses).",
                out_topic_.c_str(), out.poses.size());
  }

  // Params
  std::string in_topic_, out_topic_, map_frame_;
  bool force_map_frame_{true}, seed_from_amcl_{true};

  // AMCL seed
  std::atomic<bool> have_amcl_{false};
  XY start_{};

  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr out_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TspClearanceGoals>());
  rclcpp::shutdown();
  return 0;
}