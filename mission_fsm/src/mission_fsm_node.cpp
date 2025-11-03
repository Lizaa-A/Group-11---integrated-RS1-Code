#include <chrono>
#include <string>
#include <vector>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"


using namespace std::chrono_literals;

// -----------------------------------------------------
// Mission FSM Node
// -----------------------------------------------------
class MissionFsmNode : public rclcpp::Node
{
public:
  MissionFsmNode() : Node("mission_fsm")
  {
    // --------------------------- SUBSCRIPTIONS and PUBLISHERS ----------------------------

    // ************************************************
    // STATE 1: CLEARANCES SCANNER
    // ************************************************
    // if clearance goals received from perception
    in_topic_perception_ = declare_parameter<std::string>("perception_topic", "ugv/clearance_goals_unordered");
    out_topic_nav_ = declare_parameter<std::string>("nav_clearances_topic", "/perception/clearances");
    perception_sub_ = create_subscription<nav_msgs::msg::Path>(in_topic_perception_, 10, [&](nav_msgs::msg::Path::SharedPtr msg)
    { // store
        latest_path_ = *msg;
        no_clearances_ = false;
        RCLCPP_INFO(get_logger(), "FSM: got %zu clearances from %s", msg->poses.size(), in_topic_perception_.c_str());
        // FORWARD to nav pipeline right now
        if (nav_clearances_pub_) {
          nav_clearances_pub_->publish(*msg);
          RCLCPP_INFO(get_logger(), "FSM: forwarded clearances to %s", out_topic_nav_.c_str());
        }
        state_ = State::NAV_TO_GOAL;
      });
    // publisher to the topic your nav stack expects
    nav_clearances_pub_ = create_publisher<nav_msgs::msg::Path>(out_topic_nav_, 10);


    // ***********************************************
    // STATE 2: NAVIGATE TO GOAL
    // ***********************************************
    // once at goal, nav system sends status here
    mission_status_sub_ = create_subscription<std_msgs::msg::String>("/mission/status", 10, [&](std_msgs::msg::String::SharedPtr msg)
      {
        mission_status_ = msg->data;
      });


    // ***********************************************
    // STATE 3: CHECK CONDITIONS
    // ***********************************************
    goal_received_pub_ = create_publisher<std_msgs::msg::Bool>("/mission/goal_received", 10);
    planting_done_sub_ = create_subscription<std_msgs::msg::Bool>("/mission/planting_done", 10, [this](std_msgs::msg::Bool::SharedPtr msg)
    {
    planting_done_ = msg->data;
    RCLCPP_INFO(get_logger(), "FSM: planting pipeline says done = %s", planting_done_ ? "true" : "false");
    });

    // tank status subscription:
    water_low_sub_ = create_subscription<std_msgs::msg::Bool>("/water_tank/low", 10, [this](std_msgs::msg::Bool::SharedPtr msg)
    {
        water_low_ = msg->data;
        RCLCPP_INFO(get_logger(), "FSM: water low = %s", water_low_ ? "true" : "false");
    });

    water_level_sub_ = create_subscription<std_msgs::msg::Int32>("/water_tank/level_percent", 10, [this](std_msgs::msg::Int32::SharedPtr msg)
    {
        water_level_percent_ = msg->data;
        // RCLCPP_INFO(get_logger(), "FSM: water level = %d%%", water_level_percent_);
    });
    // only if you want to actively refill
    refill_client_ = create_client<std_srvs::srv::Empty>("/water_tank/refill");
    // placeholder: soil/sun conditions
    // will need fallback for bad conditions

    // ***********************************************
    // STATE 4: PLANTING
    // ***********************************************
    // once planting is complete, tell ugv to drive to next goal
    permit_next_pub_ = create_publisher<std_msgs::msg::Empty>("/mission/permit_next", 10);


    // ***********************************************
    // ALWAYS RUNNING - GUI, TELEMETRY
    // ***********************************************
    // placeholder for now

    // ***********************************************
    // STATUS updater
    // ***********************************************
    fsm_status_pub_ = create_publisher<std_msgs::msg::String>("/mission/fsm_status", 10);

    // -------------------------------------------------------
    // TIMER to tick the FSM
    // -------------------------------------------------------
    timer_ = create_wall_timer(200ms, std::bind(&MissionFsmNode::tick, this));
    publish_status("FSM: SCAN FOR GOAL CLEARANCES");
  }

private:
  // simple FSM states
  enum class State {
    SCAN_FOR_GOAL_CLEARANCES,
    NO_GOAL_CLEARANCES,
    NAV_TO_GOAL,
    CHECK_CONDITIONS,
    BAD_CONDITIONS, // specify what bad condition... via telemetry? more on GUI end
    REFILL_TANK,
    PLANTING, // will add detailed status from planting pipeline end
    REQUEST_NEXT_GOAL
    // MANUAL_PILOTING - STRETCH
    // NAV_NEW_TERRAIN - STRETCH
    // OBSTACLES_UNAVOIDABLE - STRETCH
  };

  void publish_status(const std::string& s)
  {
    std_msgs::msg::String m;
    m.data = s;
    fsm_status_pub_->publish(m);
  }


  void tick()
  {
    switch (state_)
    { 
      case State::SCAN_FOR_GOAL_CLEARANCES:
        state_scan_for_goal_clearances();
        break;
      case State::NO_GOAL_CLEARANCES:
        state_no_goal_clearances();
        break;
      case State::NAV_TO_GOAL:
        state_nav_to_goal();
        break;
      case State::CHECK_CONDITIONS:
        state_check_conditions();
        break;
      case State::BAD_CONDITIONS:
        state_bad_conditions();
        break;
      case State::REFILL_TANK:
        state_refill_tank();
        break;
      case State::PLANTING:
        state_planting();
        break;
      case State::REQUEST_NEXT_GOAL:
        state_request_next_goal();
        break;
    }
  }

  // 1) wait until perception sends us something
  void state_scan_for_goal_clearances()
  {
    if (latest_path_) {
      RCLCPP_INFO(get_logger(), "FSM: clearances received, sending to nav");
      state_ = State::NAV_TO_GOAL;
      publish_status("FSM: NAV TO GOAL");
    } else if (no_clearances_){
      RCLCPP_WARN(get_logger(), "FSM: no clearances received");
      publish_status("FSM: NO GOAL CLEARANCES");
      state_ = State::NO_GOAL_CLEARANCES;
    } else {
      RCLCPP_INFO(get_logger(), "FSM: waiting for /perception/clearances ...");
    }
  }

  // If no goal clearances received
  void state_no_goal_clearances()
  {
    RCLCPP_WARN(get_logger(), "ENDDDD: no goal clearances - cannot proceed - need new terrain mapping");
    // Here you could implement a fallback, e.g., manual piloting or aborting the mission
    // For now, we just stay in this state
  }

  // Nav to goal with received clearances
  void state_nav_to_goal()
  {
    // RCLCPP_INFO(get_logger(), "FSM: sending clearances to nav pipeline");
    // sending clearances to nav
    // when reached the first goal, status will be received in mission_status_sub_
    if (mission_status_ == "AT_GOAL") {
      // 1) tell planting pipeline that we are actually at the goal
      std_msgs::msg::Bool ok;
      ok.data = true;
      goal_received_pub_->publish(ok);
      // planting reset false flag
      planting_done_ = false;
      // check conditions
      // publish_status("FSM:CHECK_CONDITIONS");
      state_ = State::CHECK_CONDITIONS;
    } else if (mission_status_ == "ABORT") {
      RCLCPP_WARN(get_logger(), "FSM: nav aborted - requesting next goal");
      publish_status("FSM: REQUEST NEXT GOAL");
      state_ = State::REQUEST_NEXT_GOAL;
    } else {
      RCLCPP_DEBUG(get_logger(), "FSM: navigating to goal...");
    }
  }

  // placeholder for all your sensors (water, soil, sun)
  void state_check_conditions()
  {
    // right now: soil/sun not implemented
    bool soil_ok = true;
    bool sun_ok  = true;

    if (water_low_) {
      RCLCPP_WARN(get_logger(), "FSM: water low, go to REFILL_TANK");
      publish_status("FSM: REFILL TANK");
      state_ = State::REFILL_TANK;
      return;
    } else if (soil_ok && sun_ok) {
      RCLCPP_INFO(get_logger(), "FSM: conditions OK, PLANTING");
      state_ = State::PLANTING;
    } else {
      RCLCPP_WARN(get_logger(), "FSM: other conditions bad, BAD_CONDITIONS");
      state_ = State::BAD_CONDITIONS;
      publish_status("FSM: BAD CONDITIONS");
    }
  }

  void state_bad_conditions()
  {
    // go to next goal if conditions bad
    state_ = State::REQUEST_NEXT_GOAL;
    publish_status("FSM: REQUEST NEXT GOAL");

    RCLCPP_WARN(get_logger(), "FSM: bad conditions - going to REQUEST NEXT GOAL");
  }

  void state_refill_tank()
  {
    // pretend we are topping up water here using Liza service code
    if (!refill_client_->wait_for_service(0s)) {
        RCLCPP_WARN(get_logger(), "FSM: /water_tank/refill not available yet, staying here");
        return;
    }
    // after refill, go to next goal
    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    auto fut = refill_client_->async_send_request(req);
    RCLCPP_INFO(get_logger(), "FSM: asked water_tank to refill, next goal");
    water_low_ = false;  // optimistic
    state_ = State::REQUEST_NEXT_GOAL;
    publish_status("FSM: REQUEST NEXT GOAL");
  }

   // placeholder for planting operations
  void state_planting()
  {
    // we already told the pipeline “goal_received” in NAV_TO_GOAL
    // here we just wait for its final “planting_done”
    if (!planting_done_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "FSM: waiting for /mission/planting_done ...");
      return;    // stay in PLANTING state
    }
    // reset for next cycle
    planting_done_ = false;
    RCLCPP_INFO(get_logger(), "FSM: planting finished, request next goal");
    // after planting , go to next goal
    state_ = State::REQUEST_NEXT_GOAL;
    publish_status("FSM: REQUEST NEXT GOAL");
  }

  // tell goal_tracker / nav to pick the next goal
  void state_request_next_goal()
  {
    // publish to /mission/permit_next
    std_msgs::msg::Empty msg;
    permit_next_pub_->publish(msg);


    RCLCPP_INFO(get_logger(), "FSM: requested NEXT GOAL via /mission/permit_next");
    // remember when we started waiting
    // wait_start_ = now();
    mission_status_.clear();
    state_ = State::NAV_TO_GOAL;
    publish_status("FSM: NAV TO GOAL");
  }

  // --------------------------------------------------------
  // MEMBERS
  // --------------------------------------------------------
  State state_{State::SCAN_FOR_GOAL_CLEARANCES};

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr permit_next_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr water_low_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr water_level_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr refill_client_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr perception_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_clearances_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_received_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planting_done_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fsm_status_pub_;

  bool planting_done_{false};
  bool no_clearances_{false};
  bool water_low_{false};
  int  water_level_percent_{100};

  // data
//   std::optional<std::string> received_clearances_;
  std::string mission_status_;
  rclcpp::Time wait_start_;
  // param values
  std::optional<nav_msgs::msg::Path> latest_path_;
  std::string in_topic_perception_;
  std::string out_topic_nav_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionFsmNode>());
  rclcpp::shutdown();
  return 0;
}
