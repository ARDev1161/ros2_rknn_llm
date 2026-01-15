#pragma once

#include <functional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros2_rknn_llm {

struct PlannerStatus {
  enum class State {
    kUnknown,
    kSubmitted,
    kAccepted,
    kRejected,
    kSucceeded,
    kFailed,
    kCanceled,
    kTimeout
  };

  State state{State::kUnknown};
  std::string detail;
  std::string goal;
};

class PlannerClient {
public:
  struct Options {
    std::string goal_topic{"/robot_planner/goal"};
    std::string status_topic{"/robot_planner/goal_status"};
    bool publish_feedback{false};
    double timeout_sec{0.0};
  };

  explicit PlannerClient(rclcpp::Node *node, const Options &options);

  using StatusCallback = std::function<void(const PlannerStatus &)>;

  void set_status_callback(StatusCallback cb);
  bool available() const;
  std::string send_goals(const std::vector<std::string> &goals, std::string *error);
  void tick();
  bool cancel(const std::string &handle, std::string *error);

private:
  void status_cb(const std_msgs::msg::String::SharedPtr msg);
  PlannerStatus parse_status(const std::string &data) const;
  static std::string combine_goals(const std::vector<std::string> &goals);
  static std::string trim(const std::string &input);

  rclcpp::Node *node_{nullptr};
  Options options_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  StatusCallback status_cb_;
  bool in_flight_{false};
  rclcpp::Time in_flight_start_;
  std::string current_handle_;
  std::string current_goal_;
  size_t seq_{0};
};

}  // namespace ros2_rknn_llm
