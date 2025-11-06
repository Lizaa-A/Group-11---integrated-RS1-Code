// costmap_goal_filter.cpp
// INPUT : /ugv/clearance_goals_unfiltered is raw goals (you receive this)
// OUTPUT: /ugv/clearance_goals_unordered is filtered safe goals

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

class CostmapGoalFilter : public rclcpp::Node
{
public:
  CostmapGoalFilter() : Node("costmap_goal_filter")
  {
    // ---------- PARAMETERS ----------
    this->declare_parameter<std::string>("input_topic",  "/ugv/clearance_goals_unfiltered");
    this->declare_parameter<std::string>("output_topic", "/ugv/clearance_goals_unordered");
    this->declare_parameter<std::string>("global_costmap_topic", "global_costmap/costmap");
    this->declare_parameter<std::string>("global_frame", "map");
    this->declare_parameter<int>("lethal_threshold", 100);
    this->declare_parameter<int>("inflated_threshold", 99);
    this->declare_parameter<bool>("filter_inflated", true);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    global_frame_ = this->get_parameter("global_frame").as_string();
    lethal_thresh_ = this->get_parameter("lethal_threshold").as_int();
    inflated_thresh_ = this->get_parameter("inflated_threshold").as_int();
    filter_inflated_ = this->get_parameter("filter_inflated").as_bool();

    // ---------- SUBSCRIPTIONS ----------
    goal_sub_ = this->create_subscription<nav_msgs::msg::Path>(input_topic_, rclcpp::QoS(10).transient_local(), std::bind(&CostmapGoalFilter::goalCallback, this, std::placeholders::_1));

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(this->get_parameter("global_costmap_topic").as_string(), rclcpp::QoS(1).transient_local(), [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
          latest_costmap_ = msg;
        //   RCLCPP_INFO_ONCE(this->get_logger(),"Received costmap: %dx%d, res=%.3f, origin=(%.2f,%.2f)", msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y);
        });

    // ---------- PUBLISHER ----------
    filtered_pub_ = this->create_publisher<nav_msgs::msg::Path>(output_topic_, rclcpp::QoS(10).transient_local());

    rejected_pub_ = this->create_publisher<nav_msgs::msg::Path>("/ugv/clearance_goals_rejected", rclcpp::QoS(10).transient_local());

  }

private:
void goalCallback(const nav_msgs::msg::Path::SharedPtr path)
{
  if (path->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty path received means publishing empty filtered path.");
    publishEmptyPath();
    return;
  }

  if (!latest_costmap_) {
    RCLCPP_WARN(this->get_logger(), "No costmap yet means publishing ALL goals unfiltered (safe fallback).");
    filtered_pub_->publish(*path);
    return;
  }

  nav_msgs::msg::Path filtered;
  filtered.header.frame_id = global_frame_;
  filtered.header.stamp = this->now();

  nav_msgs::msg::Path rejected_path;  // To publish rejected goals
  rejected_path.header.frame_id = global_frame_;
  rejected_path.header.stamp = this->now();

  int accepted = 0, rejected = 0;

  for (const auto& ps : path->poses) {
    const double x = ps.pose.position.x;
    const double y = ps.pose.position.y;

    unsigned int mx, my;
    if (!worldToMap(x, y, mx, my)) {
      RCLCPP_WARN(this->get_logger(), "REJECTED (out of bounds): (%.2f, %.2f)", x, y);
      rejected++;
      rejected_path.poses.push_back(ps);
      continue;
    }

    const int8_t cost = latest_costmap_->data[my * latest_costmap_->info.width + mx];
    const bool blocked = (cost >= lethal_thresh_) ||  (filter_inflated_ && cost >= inflated_thresh_);

    if (blocked) {
      RCLCPP_WARN(this->get_logger(), "REJECTED (cost=%d): (%.2f, %.2f)", cost, x, y);
      rejected++;
      rejected_path.poses.push_back(ps);
    } else {
      filtered.poses.push_back(ps);
      accepted++;
      RCLCPP_DEBUG(this->get_logger(), "ACCEPTED (cost=%d): (%.2f, %.2f)", cost, x, y);
    }
  }

  // Publish both
  filtered_pub_->publish(filtered);
  rejected_pub_->publish(rejected_path);

  // === FINAL SUMMARY ===
  RCLCPP_INFO(this->get_logger(), 
 "=== FILTER SUMMARY ===\n"
 "  Accepted: %d\n"
 "  REJECTED: %d\n"
 "  Safe goals published: %zu\n"
 "  Rejected goals: /ugv/clearance_goals_rejected",
  accepted, rejected, filtered.poses.size());
}

  void publishEmptyPath()
  {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = global_frame_;
    empty.header.stamp    = this->now();
    filtered_pub_->publish(empty);
    RCLCPP_INFO(this->get_logger(), "Published EMPTY filtered path on %s", output_topic_.c_str());
  }

  bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my)
  {
    if (!latest_costmap_) return false;

    const auto& m = *latest_costmap_;
    const double px = (wx - m.info.origin.position.x) / m.info.resolution;
    const double py = (wy - m.info.origin.position.y) / m.info.resolution;

    if (px < 0 || py < 0 || px >= m.info.width || py >= m.info.height) {
      return false;
    }

    mx = static_cast<unsigned int>(px);
    my = static_cast<unsigned int>(py);
    return true;
  }

  // ---------- ROS ----------
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr filtered_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rejected_pub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_{nullptr};

  std::string input_topic_, output_topic_, global_frame_;
  int lethal_thresh_, inflated_thresh_;
  bool filter_inflated_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapGoalFilter>());
  rclcpp::shutdown();
  return 0;
}