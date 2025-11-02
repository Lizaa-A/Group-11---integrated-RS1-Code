// rotating_clearance_detector.cpp
//
// Node that rotates the UGV in fixed angular sectors,
// waits for a stable depth cloud at each heading,
// extracts 0 to 6 “clear” goal points from the cloud,
// and finally publishes them all as a nav_msgs/Path.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <string>
#include <optional>
#include <cmath>

class RotatingClearanceDetector : public rclcpp::Node
{
public:
  // ------------------------------------------------------------
  // Constructor: declare params, create pubs/subs, start timer
  // ------------------------------------------------------------
  RotatingClearanceDetector() : rclcpp::Node("rotating_clearance_detector"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // 1) declare parameters that can be fine tuened, especially free_range_m and rotate_speed, and min clear columns
    this->declare_parameter<std::string>("pointcloud_topic", "/camera/depth/points");
    this->declare_parameter<std::string>("camera_frame", "camera_link");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("target_frame", "map");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<double>("sector_width_rad", 2.094);  // 120 deg
    this->declare_parameter<int>("sectors", 3);
    this->declare_parameter<int>("max_goals_per_sector", 2);
    this->declare_parameter<int>("min_clear_columns", 6);
    this->declare_parameter<int>("num_columns", 180);
    this->declare_parameter<double>("max_xy", 4.0);
    this->declare_parameter<double>("obstacle_z_thresh", 0.5);
    this->declare_parameter<double>("free_range_m", 3.0);
    this->declare_parameter<double>("rotate_speed", 0.8);
    this->declare_parameter<double>("rotate_settle_s", 0.7);

    // 2) read parameters
    pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    sector_width_ = this->get_parameter("sector_width_rad").as_double();
    sectors_ = this->get_parameter("sectors").as_int();
    max_goals_sector_ = this->get_parameter("max_goals_per_sector").as_int();
    min_clear_cols_ = this->get_parameter("min_clear_columns").as_int();
    num_columns_ = this->get_parameter("num_columns").as_int();
    max_xy_ = this->get_parameter("max_xy").as_double();
    obstacle_z_thresh_ = this->get_parameter("obstacle_z_thresh").as_double();
    free_range_m_ = this->get_parameter("free_range_m").as_double();
    rotate_speed_ = this->get_parameter("rotate_speed").as_double();
    rotate_settle_s_ = this->get_parameter("rotate_settle_s").as_double();

    // 3) create a latched path publisher so markers will connect (also transient local)
    auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/ugv/clearance_goals_unordered", latched_qos);

    // 4) cmd_vel publisher
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    // 5) point cloud subscriber
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 10, std::bind(&RotatingClearanceDetector::cloudCb, this, std::placeholders::_1));

    // 6) periodic timer that drives the little state machine
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&RotatingClearanceDetector::tick, this));

    RCLCPP_INFO(this->get_logger(), "Rotating clearance detector ready.");
  }

private:
  // ------------------------------------------------------------
  // tick(): main state machine
  // - rotate to current sector
  // - stop + settle
  // - use cloud to extract goals
  // - go to next sector
  // ------------------------------------------------------------
  void tick()
  {
    if (done_) {
      return;
    }

    // desired yaw for this sector
    const double target_yaw = current_sector_ * sector_width_;

    // 1) rotate until we hit target yaw
    if (!at_yaw(target_yaw)) {
      rotate_to(target_yaw);
      waiting_for_settle_ = false;   // were moving again
      return;
    }

    // 2) we are at yaw , means now we start waiting period (only once)
    if (!waiting_for_settle_) {
      reached_yaw_time_ = this->now();
      waiting_for_settle_ = true;
      stop_robot();  // important: stop before taking cloud
      return;
    }

    // 3) how long to wait for?
    if ((this->now() - reached_yaw_time_).seconds() < rotate_settle_s_) {
      return;
    }

    // 4) now require a cloud that was received while we were waiting
    if (!latest_cloud_) {
      return;
    }

    // 5) process cloud, and extract goals for this sector
    auto sector_goals = process_cloud_for_sector(*latest_cloud_);
    latest_cloud_.reset();

    // append to global list
    for (auto & g : sector_goals) {
      all_goals_.push_back(g);
      if ((int)all_goals_.size() >= sectors_ * max_goals_sector_) {
        break;
      }
    }

    // 6) next sector
    current_sector_++;
    waiting_for_settle_ = false;

    // 7) are we done with all 3 sectors?
    if (current_sector_ >= sectors_ || (int)all_goals_.size() >= sectors_ * max_goals_sector_) {
      publish_path();
      stop_robot();
      done_ = true;
      RCLCPP_INFO(this->get_logger(), "Finished all sectors, published %zu goals.", all_goals_.size());
    }
  }

  // ------------------------------------------------------------
  // cloudCb(): store the latest point cloud
  // ------------------------------------------------------------
  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_cloud_ = msg;
  }

  // ------------------------------------------------------------
  // at_yaw(target): check TF (target_frame --> base_frame) and compare yaw
  // ------------------------------------------------------------
  bool at_yaw(double target_yaw)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(target_frame_, base_frame_, tf2::TimePointZero);
    } catch (const std::exception &e) {
      // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF %s -> %s not ready: %s",target_frame_.c_str(), base_frame_.c_str(), e.what());
      return false;
    }

    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); // roll, pitch, yaw in radians

    double diff = normalize_angle(target_yaw - yaw); // normalise to [-pi, pi]
    return std::fabs(diff) < 0.05;  // ~3 deg tolerance
  }

  // ------------------------------------------------------------
  // rotate_to(target): publish a Twist to turn on the spot
  // ------------------------------------------------------------
  void rotate_to(double target_yaw)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(target_frame_, base_frame_, tf2::TimePointZero);
    } catch (...) {
      return;
    }

    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double diff = normalize_angle(target_yaw - yaw);

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = (diff > 0 ? 1.0 : -1.0) * rotate_speed_; // positive = turn left, negative = turn right
    cmd_pub_->publish(cmd);

    RCLCPP_INFO(get_logger(), "Rotating to sector %d, cmd.z=%.2f", current_sector_, cmd.angular.z);
  }

  // ------------------------------------------------------------
  // stop_robot(): publish zero twist
  // ------------------------------------------------------------
  void stop_robot()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);  // default constructed = zeros
  }

  // ------------------------------------------------------------
  // process_cloud_for_sector():
  //   - break sector into columns
  //   - mark columns blocked if low obstacle is seen
  //   - find long runs of free columns into goals
  // ------------------------------------------------------------
  std::vector<geometry_msgs::msg::PoseStamped>
  process_cloud_for_sector(const sensor_msgs::msg::PointCloud2 &cloud)
  {
    std::vector<geometry_msgs::msg::PoseStamped> out;
    std::vector<bool> free_col(num_columns_, true);

    const double half    = sector_width_ / 2.0;
    const double ang_min = -half;
    const double ang_max = +half;

    // iterate over all points
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      // assume camera forward = +X, left = +Y
      const double angle = std::atan2(y, x);
      if (angle < ang_min || angle > ang_max) {
        continue;  // outside this sector
      }

      // ignore tall stuff above threshold: still counts as obstacle
      if (z > obstacle_z_thresh_) {
        const int col = angle_to_col(angle, ang_min, ang_max);
        if (0 <= col && col < num_columns_) {
          free_col[col] = false;
        }
        continue;
      }

      // distance in camera frame
      const double dist = std::sqrt(x*x + y*y);
      if (dist < 0.1) {
        continue;
      }

      if (dist < free_range_m_) {
        // inside free range means we keep it free (we arrived here without finding an obstacle)
      } else {
        // beyond free range means we ignore
      }
    }

    // find contiguous free runs
    int start = -1;
    for (int i = 0; i <= num_columns_; ++i) {
      const bool free_here = (i < num_columns_) ? free_col[i] : false; // last column is always false
      if (free_here) {
        if (start == -1) start = i;
      } else {
        if (start != -1) {
          const int len = i - start;
          if (len >= min_clear_cols_) {
            const int mid = start + len/2; // middle of the run
            auto maybe_pose = col_to_goal(mid, ang_min, ang_max); // convert to goal pose
            if (maybe_pose) {
              out.push_back(*maybe_pose);
              if ((int)out.size() >= max_goals_sector_) {
                break;
              }
            }
          }
          start = -1; // reset start for next run for next sector
        }
      }
    }

    RCLCPP_INFO(this->get_logger(), "Sector %d: found %zu goals", current_sector_, out.size());
    return out;
  }

  // ------------------------------------------------------------
  // angle_to_col(): map angle in [ang_min, ang_max] → column index
  // ------------------------------------------------------------
  int angle_to_col(double angle, double ang_min, double ang_max)
  {
    const double norm = (angle - ang_min) / (ang_max - ang_min); // 0..1
    int col = static_cast<int>(norm * num_columns_);
    if (col < 0) col = 0;
    if (col >= num_columns_) col = num_columns_ - 1;
    return col;
  }

  // ------------------------------------------------------------
  // col_to_goal():
  //   - pick a point at free_range_m_ in camera frame at that column angle
  //   - transform to target_frame_ (map)
  //   - clamp radius to (3,3,0)
  //   - return pose
  // ------------------------------------------------------------
  std::optional<geometry_msgs::msg::PoseStamped>
  col_to_goal(int col, double ang_min, double ang_max)
  {
    const double norm  = static_cast<double>(col) / static_cast<double>(num_columns_);
    const double angle = ang_min + norm * (ang_max - ang_min);

    // point in camera frame, at fixed distance
    const double xc = free_range_m_ * std::cos(angle);
    const double yc = free_range_m_ * std::sin(angle);
    const double zc = 0.0;

    geometry_msgs::msg::PointStamped pc_cam, pc_map;
    pc_cam.header.frame_id = camera_frame_;
    pc_cam.point.x = xc;
    pc_cam.point.y = yc;
    pc_cam.point.z = zc;

    try {
      // super important line: transform point from camera frame to target frame (map), because TF knows the chain. Camera_link to base_link to odom to map
      pc_map = tf_buffer_.transform(pc_cam, target_frame_, tf2::durationFromSec(0.05));
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "TF camera->%s failed: %s", target_frame_.c_str(), e.what());
      return std::nullopt;
    }

    // clamp radius
    double x = pc_map.point.x;
    double y = pc_map.point.y;
    double r = std::sqrt(x*x + y*y);

    const double max_r = max_xy_;
    const double min_r = 0.7;

    if (r < min_r) {
      return std::nullopt;   // too close
    }

    if (r > max_r && r > 1e-3) {
      const double s = max_r / r;
      x *= s;
      y *= s;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = target_frame_;
    pose.header.stamp = this->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;   // no rotation
    return pose;
  }

  // ------------------------------------------------------------
  // publish_path(): publish all collected goals
  // ------------------------------------------------------------
  void publish_path()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = target_frame_;
    path.header.stamp = this->now();
    path.poses = all_goals_;
    path_pub_->publish(path);
  }

  // ------------------------------------------------------------
  // normalize_angle(): wrap to [-pi, pi]. Keeps it cleaner
  // ------------------------------------------------------------
  static double normalize_angle(double a)
  {
    while (a > M_PI)  a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  }

  // ------------------ members ------------------
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // settle state
  rclcpp::Time reached_yaw_time_;
  bool waiting_for_settle_{false};

  // params
  std::string pointcloud_topic_, camera_frame_, base_frame_, target_frame_, cmd_vel_topic_;
  int sectors_{}, max_goals_sector_{}, min_goals_sector_{}, num_columns_{}, min_clear_cols_{};
  double sector_width_{}, max_xy_{}, obstacle_z_thresh_{}, free_range_m_{}, rotate_speed_{}, rotate_settle_s_{};

  // state
  int current_sector_{0};
  bool done_{false};
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::vector<geometry_msgs::msg::PoseStamped> all_goals_;
};

// ------------------------------------------------------------
// main(): start node
// ------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RotatingClearanceDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
