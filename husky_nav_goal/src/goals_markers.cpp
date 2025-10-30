#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"

// Node to visualise goals as markers in RViz (and relevant GUI)
class GoalsMarkers : public rclcpp::Node {
public:
  GoalsMarkers() : rclcpp::Node("goals_markers")
  {
    in_topic_ = declare_parameter<std::string>("in_topic", "/mission/waypoints"); // subscribe to goals here
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/mission/markers"); // publish markers here
    
    show_text_ = declare_parameter<bool>("show_text", true); // numbered labels for goals
    z_offset_ = declare_parameter<double>("z_offset", 0.1); // raise markers a bit
    scale_xyz_ = declare_parameter<double>("scale", 0.25); // sphere size m
    text_scale_ = declare_parameter<double>("text_scale", 0.25); // text height m

    // Colour for the markers
    color_r_ = declare_parameter<double>("color_r", 0.0);
    color_g_ = declare_parameter<double>("color_g", 0.7);
    color_b_ = declare_parameter<double>("color_b", 1.0);
    color_a_ = declare_parameter<double>("color_a", 1.0);

    // Marker publisher: transient_local so RViz can see last published state
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    // Create the publisher
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, qos);

    // Path subscriber (your goals as PoseStamped[])
    path_sub_ = create_subscription<nav_msgs::msg::Path>(in_topic_, rclcpp::QoS(1).reliable().transient_local(), std::bind(&GoalsMarkers::onPath, this, std::placeholders::_1));
    // RCLCPP_INFO(get_logger(), "Listening on '%s', publishing markers on '%s'", in_topic_.c_str(), marker_topic_.c_str());
  }

private:
  // Logic to handle incoming Path messages/goals
  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // Ignore empty goals
    if (!msg || msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty Path; skipping markers.");
      return;
    }

    // Prepare MarkerArray
    visualization_msgs::msg::MarkerArray array;

    // Spheres as a single SPHERE_LIST marker
    visualization_msgs::msg::Marker spheres;
    spheres.header = msg->header; // frame_id should be "map"
    spheres.ns = "goals"; // namespace
    spheres.id = 0; // single marker
    spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST; // multiple spheres in one marker
    spheres.action = visualization_msgs::msg::Marker::ADD; // add/modify
    spheres.pose.orientation.w = 1.0;
    spheres.lifetime = rclcpp::Duration(0,0);     // forever until replaced
    // Sphere diameter and colour
    spheres.scale.x = scale_xyz_;
    spheres.scale.y = scale_xyz_;
    spheres.scale.z = scale_xyz_;
    spheres.color.r = color_r_;
    spheres.color.g = color_g_;
    spheres.color.b = color_b_;
    spheres.color.a = color_a_;

    // Fill in points and per-point colors
    spheres.points.reserve(msg->poses.size());

    // Add each goal as a sphere
    for (const auto & ps : msg->poses) {
      geometry_msgs::msg::Point pt = ps.pose.position;
      pt.z += z_offset_;
      spheres.points.push_back(pt);
      std_msgs::msg::ColorRGBA c;
      c.r = color_r_; c.g = color_g_; c.b = color_b_; c.a = color_a_;
      spheres.colors.push_back(c); // optional per-point color
    }
    // Add spheres to the array
    array.markers.push_back(spheres);

    // Numbered labels
    if (show_text_) {
      int id = 1000; // text marker IDs separate from spheres
      for (size_t i = 0; i < msg->poses.size(); ++i) {
        visualization_msgs::msg::Marker text;
        text.header = msg->header;
        text.ns = "goal_labels";
        text.id = id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.scale.z = text_scale_; // text height
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        text.pose = geometry_msgs::msg::Pose{};
        text.pose.position = msg->poses[i].pose.position;
        text.pose.position.z += z_offset_ + scale_xyz_*0.8; // above the sphere
        text.text = std::to_string(i+1); // 1-based index
        array.markers.push_back(text);
      }
    }

    // Publish the MarkerArray
    marker_pub_->publish(array);
    // RCLCPP_INFO(get_logger(), "Published %zu goals as markers%s.", msg->poses.size(), show_text_ ? " + labels" : "");
  }

  // Params declaration and definition
  std::string in_topic_;
  std::string marker_topic_;
  bool show_text_{true};
  double z_offset_{0.1}, scale_xyz_{0.25}, text_scale_{0.25};
  double color_r_{0.0}, color_g_{0.7}, color_b_{1.0}, color_a_{1.0};

  // ROS I/O connection
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

// Main function temporarily here
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalsMarkers>());
  rclcpp::shutdown();
  return 0;
}
