#include <string>
#include <atomic>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PlantingSiteMarkers : public rclcpp::Node {
public:
  PlantingSiteMarkers() : rclcpp::Node("planting_site_markers"),
                          tf_buffer_(this->get_clock()),
                          tf_listener_(tf_buffer_) {
    map_frame_       = declare_parameter<std::string>("map_frame", "map");
    base_frame_      = declare_parameter<std::string>("base_frame", "base_link");
    marker_topic_    = declare_parameter<std::string>("marker_topic", "/mission/site_markers");
    tf_timeout_s_    = declare_parameter<double>("tf_timeout_s", 0.2);
    drop_on_sun_ok_  = declare_parameter<bool>("drop_on_sun_ok", false); 
    x_size_m_        = declare_parameter<double>("x_size_m", 0.6);       
    square_size_m_   = declare_parameter<double>("square_size_m", 0.35);  
    circle_diam_m_   = declare_parameter<double>("circle_diam_m", 0.5);   
    marker_z_        = declare_parameter<double>("marker_z", 0.05);       
    line_width_m_    = declare_parameter<double>("line_width_m", 0.05);   

    reject_reason_debounce_ms_ = declare_parameter<int>("reject_reason_debounce_ms", 150);
    sun_ok_max_age_s_          = declare_parameter<double>("sun_ok_max_age_s", 1.0); 

    auto qos = rclcpp::QoS(1).reliable().transient_local();
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, qos);

    sub_ready_ = create_subscription<std_msgs::msg::Bool>(
      "/soil_prep/ready_to_plant", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          dropGreenCircleAtRobot();
          saw_sun_ok_edge_ = false; 
        }
      });

    sub_reject_ = create_subscription<std_msgs::msg::Bool>(
      "/soil_prep/rejected", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        pending_reject_ = true;
        // small debounce so /sunlight/ok latest value lands
        if (!debounce_timer_) {
          debounce_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(reject_reason_debounce_ms_),
            [this](){
              if (!pending_reject_) { debounce_timer_.reset(); return; }
              pending_reject_ = false;

              const bool sun_fresh = (now() - last_sun_ok_stamp_).seconds() <= sun_ok_max_age_s_;
              if (sun_fresh && last_sun_ok_value_ == false) {
                dropYellowXAtRobot(); // sunlight inadequate
              } else {
                //soil inadequate
                dropBlueXAtRobot();
                // sunlight adequate
                if (sun_fresh && last_sun_ok_value_) 
                dropYellowSquareAtRobot();
              }
              debounce_timer_.reset();
            });
        }
      });

    sub_sun_ok_ = create_subscription<std_msgs::msg::Bool>(
      "/sunlight/ok", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        // Track latest value 
        last_sun_ok_value_ = msg->data;
        last_sun_ok_stamp_ = now();

        if (drop_on_sun_ok_) {
          if (msg->data && !saw_sun_ok_edge_) {
            saw_sun_ok_edge_ = true;
            dropYellowSquareAtRobot();
          }
          if (!msg->data) saw_sun_ok_edge_ = false;
        }
      });

    RCLCPP_INFO(get_logger(), "PlantingSiteMarkers ready. TF %s -> %s, publishing on %s",
                map_frame_.c_str(), base_frame_.c_str(), marker_topic_.c_str());
  }

private:
  bool getRobotPose(geometry_msgs::msg::Pose& out_pose) {
    try {
      auto tf = tf_buffer_.lookupTransform(
        map_frame_, base_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_s_));
      out_pose.position.x = tf.transform.translation.x;
      out_pose.position.y = tf.transform.translation.y;
      out_pose.position.z = marker_z_;
      out_pose.orientation.w = 1.0;
      return true;
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "TF lookup %s->%s failed: %s",
                           map_frame_.c_str(), base_frame_.c_str(), e.what());
      return false;
    }
  }

  visualization_msgs::msg::Marker mkHeader(uint32_t id) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = now();
    m.ns = "planting_checks";
    m.id = id;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = rclcpp::Duration(0,0); 
    return m;
  }

  visualization_msgs::msg::Marker mkGreenCircle(const geometry_msgs::msg::Pose& pose) {
    auto m = mkHeader(next_id_++);
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.pose = pose;
    m.scale.x = circle_diam_m_;
    m.scale.y = circle_diam_m_;
    m.scale.z = 0.02;
    m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 1.0f;
    return m;
  }

  visualization_msgs::msg::Marker mkYellowSquare(const geometry_msgs::msg::Pose& pose) {
    auto m = mkHeader(next_id_++);
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.pose = pose;
    m.scale.x = square_size_m_;
    m.scale.y = square_size_m_;
    m.scale.z = 0.02;
    m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 1.0f;
    return m;
  }

  visualization_msgs::msg::Marker mkX(const geometry_msgs::msg::Pose& pose, bool yellow) {
    auto m = mkHeader(next_id_++);
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.pose = pose;
    m.scale.x = line_width_m_; // line width
    geometry_msgs::msg::Point p1, p2, p3, p4;
    const double L = x_size_m_ * 0.5;
    p1.x = -L; p1.y = -L; p1.z = 0.0;
    p2.x =  L; p2.y =  L; p2.z = 0.0;
    p3.x = -L; p3.y =  L; p3.z = 0.0;
    p4.x =  L; p4.y = -L; p4.z = 0.0;
    m.points = {p1,p2,p3,p4};
    if (yellow) { m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f; }
    else        { m.color.r = 0.0f; m.color.g = 0.5f; m.color.b = 1.0f; }
    m.color.a = 1.0f;
    return m;
  }

  void publishOne(const visualization_msgs::msg::Marker& m) {
    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(m);
    pub_->publish(arr);
  }

  void dropGreenCircleAtRobot() {
    geometry_msgs::msg::Pose p;
    if (!getRobotPose(p)) return;
    publishOne(mkGreenCircle(p));
    RCLCPP_INFO(get_logger(), "Dropped GREEN circle at robot pose (soil+sun OK).");
  }

  void dropYellowSquareAtRobot() {
    geometry_msgs::msg::Pose p;
    if (!getRobotPose(p)) return;
    publishOne(mkYellowSquare(p));
    RCLCPP_INFO(get_logger(), "Dropped YELLOW square at robot pose (sunlight OK).");
  }

  void dropYellowXAtRobot() {
    geometry_msgs::msg::Pose p;
    if (!getRobotPose(p)) return;
    publishOne(mkX(p, /*yellow=*/true));
    RCLCPP_INFO(get_logger(), "Dropped YELLOW X at robot pose (sunlight inadequate).");
  }

  void dropBlueXAtRobot() {
    geometry_msgs::msg::Pose p;
    if (!getRobotPose(p)) return;
    publishOne(mkX(p, /*yellow=*/false));
    RCLCPP_INFO(get_logger(), "Dropped BLUE X at robot pose (soil inadequate).");
  }

  std::string map_frame_, base_frame_, marker_topic_;
  double tf_timeout_s_{0.2};
  bool drop_on_sun_ok_{false};
  double x_size_m_{0.6}, square_size_m_{0.35}, circle_diam_m_{0.5};
  double marker_z_{0.05}, line_width_m_{0.05};

  int reject_reason_debounce_ms_{150};
  double sun_ok_max_age_s_{1.0};

  // Sun tracking
  bool last_sun_ok_value_{false};
  rclcpp::Time last_sun_ok_stamp_{0,0,RCL_ROS_TIME};
  bool saw_sun_ok_edge_{false};

  // State
  bool pending_reject_{false};

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Pub/Sub
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ready_, sub_reject_, sub_sun_ok_;
  rclcpp::TimerBase::SharedPtr debounce_timer_;

  // IDs
  uint32_t next_id_{1};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlantingSiteMarkers>());
  rclcpp::shutdown();
  return 0;
}