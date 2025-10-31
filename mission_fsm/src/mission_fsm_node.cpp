#include <chrono>
#include <string>
#include <vector>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

// -----------------------------------------------------
// Mission FSM Node
// -----------------------------------------------------
class MissionFsmNode : public rclcpp::Node
{
public:
  MissionFsmNode() : Node("mission_fsm")
  {
    // -------------------------------------------------------
    // SUBSCRIPTIONS and PUBLISHERS
    // -------------------------------------------------------  

    // STATE 1: CLEARANCES SCANNER
    // if no clearances send msg here

    // if clearances received:
    // perception_sub_ = create_subscription<std_msgs::msg::String>("/perception/clearances", 10, [&](std_msgs::msg::String::SharedPtr msg)
    //  {
    //   received_clearances_ = msg->data;
    //   no_clearances_ = false;
    //   RCLCPP_INFO(get_logger(), "FSM: got clearances: %s", received_clearances_->c_str());
    //  });

    in_topic_perception_ = declare_parameter<std::string>(
      "perception_topic", "ugv/clearance_goals_unordered");
    out_topic_nav_ = declare_parameter<std::string>(
      "nav_clearances_topic", "/perception/clearances");

    perception_sub_ = create_subscription<nav_msgs::msg::Path>(
      in_topic_perception_, 10,
      [&](nav_msgs::msg::Path::SharedPtr msg){
        // store
        latest_path_ = *msg;
        // received_clearances_ = msg->data;
        no_clearances_ = false;
        RCLCPP_INFO(get_logger(), "FSM: got %zu clearances from %s",
                    msg->poses.size(), in_topic_perception_.c_str());
        // FORWARD to nav pipeline right now
        if (nav_clearances_pub_) {
          nav_clearances_pub_->publish(*msg);
          RCLCPP_INFO(get_logger(), "FSM: forwarded clearances → %s", out_topic_nav_.c_str());
        }

        state_ = State::NAV_TO_GOAL;
      });

    // 2) publisher to the topic your nav stack expects
    nav_clearances_pub_ = create_publisher<nav_msgs::msg::Path>(
      out_topic_nav_, 10);


    // STATE 2: NAVIGATE TO GOAL
    // once at goal, nav system sends status here
    mission_status_sub_ = create_subscription<std_msgs::msg::String>("/mission/status", 10, [&](std_msgs::msg::String::SharedPtr msg)
      {
        mission_status_ = msg->data;
      });

    // STATE 3: CHECK CONDITIONS
    // placeholder: water/soil/sun conditions
    // will need fallback for bad conditions

    // STATE 4: PLANTING
    // once planting is complete, tell ugv to drive to next goal
    permit_next_pub_ = create_publisher<std_msgs::msg::Empty>("/mission/permit_next", 10);

    // ALWAYS RUNNING - GUI, TELEMETRY
    // placeholder for now



    // -------------------------------------------------------
    // TIMER to tick the FSM
    // -------------------------------------------------------
    timer_ = create_wall_timer(200ms, std::bind(&MissionFsmNode::tick, this));
  }

private:
  // simple FSM states
  enum class State {
    SCAN_FOR_GOAL_CLEARANCES,
    NO_GOAL_CLEARANCES,
    NAV_TO_GOAL,
    CHECK_CONDITIONS,
    BAD_CONDITIONS,
    REFILL_TANK,
    PLANTING,
    REQUEST_NEXT_GOAL
    // MANUAL_PILOTING - STRETCH
    // NAV_NEW_TERRAIN - STRETCH
    // OBSTACLES_UNAVOIDABLE - STRETCH
  };

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
    } else if (no_clearances_){
      RCLCPP_WARN(get_logger(), "FSM: no clearances received");
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
    RCLCPP_INFO(get_logger(), "FSM: sending clearances to nav pipeline");
    // sending clearances to nav
    // when reached the first goal, status will be received in mission_status_sub_
    if (mission_status_ == "AT_GOAL") {
      // check conditions
      state_ = State::CHECK_CONDITIONS;
    } else if (mission_status_ == "ABORT") {
      RCLCPP_WARN(get_logger(), "FSM: nav aborted - requesting next goal");
      state_ = State::REQUEST_NEXT_GOAL;
    } else {
      RCLCPP_DEBUG(get_logger(), "FSM: navigating to goal...");
    }
  }

  // placeholder for all your sensors (water, soil, sun)
  void state_check_conditions()
  {
    // TODO: read /water_tank/low, /soil/ok, /sun/ok, etc.
    // for now: always OK
    bool conditions_ok = true; // placeholder
    // bool water_low = false;  // placeholder

    if (conditions_ok) {
      RCLCPP_INFO(get_logger(), "FSM: conditions OK, go to PLANTING");
      state_ = State::PLANTING;
    } else {
      RCLCPP_WARN(get_logger(), "FSM: conditions NOT OK, depends on what's wrong. Either BAD_CONDITIONS or REFILL_TANK");
      if (water_low_) {
        state_ = State::REFILL_TANK;
      } else {
        state_ = State::BAD_CONDITIONS;
      }
    }
  } 

  void state_bad_conditions()
  {
    // go to next goal if conditions bad
    state_ = State::REQUEST_NEXT_GOAL;
    RCLCPP_WARN(get_logger(), "FSM: bad conditions - going to REQUEST NEXT GOAL");
  }

  void state_refill_tank()
  {
    // placeholder for refill logic
    RCLCPP_INFO(get_logger(), "ENDDDD: refilling water tank...");
    
    // after refill, go to next goal
    // state_ = State::REQUEST_NEXT_GOAL;
  }

   // placeholder for planting operations
  void state_planting()
  {
    // TODO: call service, publish tool command, etc.
    RCLCPP_INFO(get_logger(), "FSM: (placeholder) performing planting operation...");
    // after planting → go to next goal
    state_ = State::REQUEST_NEXT_GOAL;
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
    // state_ = State::SCAN_FOR_GOAL_CLEARANCES;
  }

  // --------------------------------------------------------
  // MEMBERS
  // --------------------------------------------------------
  State state_{State::SCAN_FOR_GOAL_CLEARANCES};

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr  permit_next_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_status_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr perception_pub_;
//   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_clearances_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr perception_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr    nav_clearances_pub_;

  bool no_clearances_{false};
  bool water_low_{false};
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
