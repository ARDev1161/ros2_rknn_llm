#include "ros2_rknn_llm/planner_client.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace ros2_rknn_llm {

PlannerClient::PlannerClient(rclcpp::Node *node, const Options &options)
: node_(node), options_(options)
{
  goal_pub_ = node_->create_publisher<std_msgs::msg::String>(options_.goal_topic, 10);
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    options_.status_topic, 10,
    std::bind(&PlannerClient::status_cb, this, std::placeholders::_1));
}

void PlannerClient::set_status_callback(StatusCallback cb)
{
  status_cb_ = std::move(cb);
}

bool PlannerClient::available() const
{
  return goal_pub_ && status_sub_;
}

std::string PlannerClient::send_goals(const std::vector<std::string> &goals, std::string *error)
{
  if (goals.empty()) {
    if (error) {
      *error = "goals list is empty";
    }
    return "";
  }
  if (goal_pub_->get_subscription_count() == 0) {
    if (error) {
      *error = "planner goal topic has no subscribers";
    }
    return "";
  }

  std_msgs::msg::String msg;
  current_goal_ = combine_goals(goals);
  msg.data = current_goal_;
  goal_pub_->publish(msg);

  in_flight_ = true;
  in_flight_start_ = node_->now();
  current_handle_ = std::to_string(++seq_);

  return current_handle_;
}

void PlannerClient::tick()
{
  if (!in_flight_) {
    return;
  }
  if (options_.timeout_sec <= 0.0) {
    return;
  }
  const auto elapsed = (node_->now() - in_flight_start_).seconds();
  if (elapsed < options_.timeout_sec) {
    return;
  }

  in_flight_ = false;
  PlannerStatus status;
  status.state = PlannerStatus::State::kTimeout;
  status.detail = "timeout";
  status.goal = current_goal_;
  if (status_cb_) {
    status_cb_(status);
  }
}

bool PlannerClient::cancel(const std::string &handle, std::string *error)
{
  (void)handle;
  if (error) {
    *error = "cancel not supported by goal gateway";
  }
  return false;
}

void PlannerClient::status_cb(const std_msgs::msg::String::SharedPtr msg)
{
  PlannerStatus status = parse_status(msg->data);
  status.goal = current_goal_;

  if (status.state == PlannerStatus::State::kRejected ||
      status.state == PlannerStatus::State::kAccepted ||
      status.state == PlannerStatus::State::kFailed ||
      status.state == PlannerStatus::State::kSucceeded ||
      status.state == PlannerStatus::State::kCanceled) {
    in_flight_ = false;
  }

  if (status_cb_) {
    status_cb_(status);
  }
}

PlannerStatus PlannerClient::parse_status(const std::string &data) const
{
  PlannerStatus status;
  std::string trimmed = trim(data);
  if (trimmed.empty()) {
    status.state = PlannerStatus::State::kUnknown;
    return status;
  }

  std::string key = trimmed;
  std::string detail;
  auto colon_pos = trimmed.find(':');
  if (colon_pos != std::string::npos) {
    key = trim(trimmed.substr(0, colon_pos));
    detail = trim(trimmed.substr(colon_pos + 1));
  }

  std::string lower = key;
  std::transform(lower.begin(), lower.end(), lower.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (lower == "submitted") {
    status.state = PlannerStatus::State::kSubmitted;
  } else if (lower == "accepted") {
    status.state = PlannerStatus::State::kAccepted;
  } else if (lower == "rejected") {
    status.state = PlannerStatus::State::kRejected;
  } else if (lower == "failed") {
    status.state = PlannerStatus::State::kFailed;
  } else if (lower == "succeeded" || lower == "success") {
    status.state = PlannerStatus::State::kSucceeded;
  } else if (lower == "canceled" || lower == "cancelled") {
    status.state = PlannerStatus::State::kCanceled;
  } else {
    status.state = PlannerStatus::State::kUnknown;
  }

  status.detail = detail;
  return status;
}

std::string PlannerClient::combine_goals(const std::vector<std::string> &goals)
{
  if (goals.size() == 1) {
    return trim(goals.front());
  }

  std::ostringstream stream;
  stream << "(and";
  for (const auto &goal : goals) {
    const std::string trimmed = trim(goal);
    if (!trimmed.empty()) {
      stream << " " << trimmed;
    }
  }
  stream << ")";
  return stream.str();
}

std::string PlannerClient::trim(const std::string &input)
{
  auto begin = input.begin();
  auto end = input.end();
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  begin = std::find_if(begin, end, not_space);
  auto rbegin = std::find_if(input.rbegin(), input.rend(), not_space).base();
  if (begin >= rbegin) {
    return "";
  }
  return std::string(begin, rbegin);
}

}  // namespace ros2_rknn_llm
