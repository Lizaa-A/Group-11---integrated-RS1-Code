// Measuring the weight of the seeds

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"   // NEW
#include <deque>
#include <cmath>
#include <limits>
#include <algorithm>
#include <type_traits>

#ifdef __INTELLISENSE__
  #include <type_traits>
  namespace std {
    template<class From, class To>
    inline constexpr bool is_convertible_v = std::is_convertible<From, To>::value;
  }
#endif

class PlantingWeight : public rclcpp::Node {
public:
  PlantingWeight() : Node("planting_weight")
  {
    // parameters
    raw_zero_          = declare_parameter<double>("raw_zero", 0.0);
    raw_known_         = declare_parameter<double>("raw_known", 10000.0);
    known_weight_g_    = declare_parameter<double>("known_weight_g", 1000.0);
    empty_holder_g_    = declare_parameter<double>("empty_holder_g", 120.0);
    seed_mass_g_       = declare_parameter<double>("seed_mass_g", 0.040);
    low_threshold_     = declare_parameter<int>("low_threshold", 50);
    smooth_window_     = declare_parameter<int>("smooth_window", 10);
    publish_step_g_    = declare_parameter<double>("publish_step_g", 5.0);
    raw_type_          = declare_parameter<std::string>("raw_msg_type", "float32");
    // stop service name (so you can remap if needed)
    stop_srv_name_     = declare_parameter<std::string>("stop_service", "/plant/stop"); // NEW

    // subscriptions
    if (raw_type_ == "int32") {
      raw_sub_i32_ = create_subscription<std_msgs::msg::Int32>(
        "/seed_weight/load_cell_raw", 10,
        [this](std_msgs::msg::Int32::SharedPtr m){ on_raw(static_cast<double>(m->data)); });
      RCLCPP_INFO(get_logger(), "Subscribing to /seed_weight/load_cell_raw as Int32");
    } else {
      raw_sub_f32_ = create_subscription<std_msgs::msg::Float32>(
        "/seed_weight/load_cell_raw", 10,
        [this](std_msgs::msg::Float32::SharedPtr m){ on_raw(static_cast<double>(m->data)); });
      RCLCPP_INFO(get_logger(), "Subscribing to /seed_weight/load_cell_raw as Float32");
    }

    // latched publishers
    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pub_weight_g_ = create_publisher<std_msgs::msg::Float32>("/seed_weight/weight_g", latched);
    pub_percent_  = create_publisher<std_msgs::msg::Float32>("/seed_weight/percent_full", latched);
    pub_low_      = create_publisher<std_msgs::msg::Bool>   ("/seed_weight/low", latched);
    pub_empty_    = create_publisher<std_msgs::msg::Bool>   ("/seed_weight/empty", latched);

    // status stream
    pub_status_   = create_publisher<std_msgs::msg::String> ("/mission/fsm_status", 10);

    // service client to stop planting
    stop_cli_ = create_client<std_srvs::srv::Trigger>(stop_srv_name_); // NEW

    // loop
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&PlantingWeight::tick, this));
  }

private:
  void save_param(const char* name, double v){
    rclcpp::Parameter p(name, v);
    this->set_parameter(p);
  }

  void announce(const std::string& s){
    std_msgs::msg::String m; m.data = s;
    pub_status_->publish(m);
  }

  void on_raw(double raw){
    if (smooth_window_ < 1) smooth_window_ = 1;
    raw_buf_.push_back(raw);
    if ((int)raw_buf_.size() > smooth_window_) raw_buf_.pop_front();

    double sum = 0.0;
    for (double x : raw_buf_) sum += x;
    avg_raw_   = sum / static_cast<double>(raw_buf_.size());
    avg_count_ = static_cast<int>(raw_buf_.size());
  }

  double raw_to_grams(double raw) const {
    const double denom = (raw_known_ - raw_zero_);
    if (std::abs(denom) < 1e-9) return 0.0;
    const double scale = known_weight_g_ / denom;
    double w = (raw - raw_zero_) * scale;
    return std::max(0.0, w);
  }

  void tick(){
    if (avg_count_ == 0) return;

    const double weight_g = raw_to_grams(avg_raw_);
    const double net_g    = std::max(0.0, weight_g - empty_holder_g_);

    int seeds = 0;
    if (seed_mass_g_ > 0.0) {
      seeds = static_cast<int>(std::floor(net_g / seed_mass_g_ + 1e-6));
      seeds = std::max(0, seeds);
    }

    const bool low_now   = (seeds <= low_threshold_);
    const bool empty_now = (seeds == 0);

    float percent_full = 0.0f;
    if (known_weight_g_ > 0.0)
      percent_full = static_cast<float>(std::clamp(100.0 * net_g / known_weight_g_, 0.0, 100.0));

    bool should_pub = false;
    if (std::isnan(last_weight_g_)) should_pub = true;
    else if (std::abs(weight_g - last_weight_g_) >= publish_step_g_) should_pub = true;
    else if (low_now   != last_low_)   should_pub = true;
    else if (empty_now != last_empty_) should_pub = true;

    if (should_pub) {
      std_msgs::msg::Float32 w; w.data = static_cast<float>(weight_g); pub_weight_g_->publish(w);
      std_msgs::msg::Float32 p; p.data = percent_full;                 pub_percent_->publish(p);
      std_msgs::msg::Bool    b; b.data = low_now;                      pub_low_->publish(b);
      std_msgs::msg::Bool    e; e.data = empty_now;                    pub_empty_->publish(e);

      if (low_now && !last_low_)     announce("FSM: LOW SEED LEVEL");
      if (empty_now && !last_empty_) {
        announce("FSM: SEEDS EMPTY — requesting plant stop");
        // fire-and-forget Trigger to stop planting
        if (stop_cli_) {
          if (stop_cli_->service_is_ready() || stop_cli_->wait_for_service(std::chrono::milliseconds(1))) {
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            (void)stop_cli_->async_send_request(req); // non-blocking
          } else {
            RCLCPP_WARN(get_logger(), "Stop service '%s' not available yet", stop_srv_name_.c_str());
          }
        }
      }

      last_weight_g_ = weight_g;
      last_low_      = low_now;
      last_empty_    = empty_now;

      RCLCPP_INFO(get_logger(),
                  "SEED WEIGHT: weight=%.1fg net=%.1fg (%.1f%%) low=%s empty=%s raw≈%.1f",
                  weight_g, net_g, percent_full,
                  (low_now ? "true" : "false"),
                  (empty_now ? "true" : "false"), avg_raw_);
    }
  }

  // subs/pubs
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   raw_sub_i32_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr raw_sub_f32_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_weight_g_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_percent_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_low_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_empty_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_status_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr     srv_tare_, srv_set_cal2_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr    stop_cli_;     // NEW
  rclcpp::TimerBase::SharedPtr                         timer_;

  // state
  std::deque<double> raw_buf_;
  double avg_raw_{0.0};
  int    avg_count_{0};

  // params
  double raw_zero_;
  double raw_known_;
  double known_weight_g_;
  double empty_holder_g_;
  double seed_mass_g_;
  int    low_threshold_;
  int    smooth_window_;
  double publish_step_g_;
  std::string raw_type_;
  std::string stop_srv_name_; // NEW

  // last snapshot
  double last_weight_g_{std::numeric_limits<double>::quiet_NaN()};
  bool   last_low_{false};
  bool   last_empty_{false};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlantingWeight>());
  rclcpp::shutdown();
  return 0;
}