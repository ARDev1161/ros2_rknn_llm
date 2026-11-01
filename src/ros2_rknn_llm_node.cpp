#include <algorithm>
#include <chrono>
#include <cctype>
#include <deque>
#include <fstream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nlohmann/json.hpp"
#if __has_include(<nlohmann/json-schema.hpp>)
#include <nlohmann/json-schema.hpp>
#define HAVE_JSON_SCHEMA 1
#else
#define HAVE_JSON_SCHEMA 0
#endif
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ros2_rknn_llm/planner_client.hpp"
#include "ros2_rknn_llm/rkllm_engine.hpp"

namespace ros2_rknn_llm {

using json = nlohmann::json;
#if HAVE_JSON_SCHEMA
using nlohmann::json_schema::json_validator;
#endif
using namespace std::chrono_literals;

struct ParsedResponse {
  std::string type{"chat"};
  std::string reply;
  std::vector<std::string> goals;
  std::string raw;
};

struct NodeEvent {
  enum class Kind { kLlmPartial, kLlmFinish, kLlmError, kPlannerStatus };
  Kind kind{Kind::kLlmPartial};
  std::string text;
  PlannerStatus planner_status;
};

class RkllmNode : public rclcpp::Node {
public:
  RkllmNode()
  : Node("rkllm_node")
  {
    model_path_ = declare_parameter<std::string>("model_path", "");
    problem_pddl_topic_ = declare_parameter<std::string>("problem_pddl_topic", "/pddl/problem");
    domain_file_ = declare_parameter<std::string>("domain_file", "pddl/domain.pddl");
    publish_partials_ = declare_parameter<bool>("publish_partials", true);
    max_pddl_chars_ = declare_parameter<int>("max_pddl_chars", 12000);
    max_new_tokens_ = declare_parameter<int>("max_new_tokens", 256);
    max_context_len_ = declare_parameter<int>("max_context_len", 2048);
    top_p_ = declare_parameter<double>("top_p", 0.95);
    temperature_ = declare_parameter<double>("temperature", 0.8);
    repeat_penalty_ = declare_parameter<double>("repeat_penalty", 1.1);
    keep_history_ = declare_parameter<int>("keep_history", 0);
    skip_special_token_ = declare_parameter<bool>("skip_special_token", true);
    lora_model_path_ = declare_parameter<std::string>("lora_model_path", "");
    lora_adapter_name_ = declare_parameter<std::string>("lora_adapter_name", "default");
    lora_scale_ = declare_parameter<double>("lora_scale", 1.0);
    prompt_cache_path_ = declare_parameter<std::string>("prompt_cache_path", "");
    save_prompt_cache_ = declare_parameter<bool>("save_prompt_cache", false);
    system_prompt_file_ = declare_parameter<std::string>("system_prompt_file", "config/system_prompt.json");
    busy_mode_ = declare_parameter<std::string>("busy_mode", "reject");
    publish_planner_feedback_ = declare_parameter<bool>("publish_planner_feedback", false);
    planner_timeout_sec_ = declare_parameter<double>("planner_timeout_sec", 0.0);
    planner_goal_topic_ = declare_parameter<std::string>("planner_goal_topic", "/robot_planner/goal");
    planner_status_topic_ = declare_parameter<std::string>("planner_status_topic", "/robot_planner/goal_status");

    if (model_path_.empty()) {
      throw std::runtime_error("model_path parameter is required");
    }

    assistant_pub_ = create_publisher<std_msgs::msg::String>("~/assistant_text", 10);
    partial_pub_ = create_publisher<std_msgs::msg::String>("~/partial", 10);

    user_sub_ = create_subscription<std_msgs::msg::String>(
      "~/user_text", 10, std::bind(&RkllmNode::on_user_text, this, std::placeholders::_1));
    problem_sub_ = create_subscription<std_msgs::msg::String>(
      problem_pddl_topic_, 10,
      std::bind(&RkllmNode::on_problem_pddl, this, std::placeholders::_1));

    clear_cache_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_cache",
      std::bind(&RkllmNode::on_clear_cache, this, std::placeholders::_1, std::placeholders::_2));

    PlannerClient::Options planner_options;
    planner_options.goal_topic = planner_goal_topic_;
    planner_options.status_topic = planner_status_topic_;
    planner_options.publish_feedback = publish_planner_feedback_;
    planner_options.timeout_sec = planner_timeout_sec_;
    planner_client_ = std::make_unique<PlannerClient>(this, planner_options);
    planner_client_->set_status_callback(
      [this](const PlannerStatus &status) { enqueue_planner_status(status); });

    system_prompt_ = load_system_prompt(system_prompt_file_);
    domain_text_ = load_domain_file(domain_file_);
    setup_validators();

    engine_ = std::make_unique<RkllmEngine>(RkllmEngine::Config{
      model_path_,
      max_new_tokens_,
      max_context_len_,
      top_p_,
      temperature_,
      repeat_penalty_,
      keep_history_,
      skip_special_token_,
      lora_model_path_,
      lora_adapter_name_,
      lora_scale_,
      prompt_cache_path_,
      save_prompt_cache_
    });

    engine_->set_event_callback([this](const LlmEvent &event) { enqueue_llm_event(event); });

    std::string error;
    if (!engine_->initialize(&error)) {
      throw std::runtime_error("RKLLM init failed: " + error);
    }
    for (const auto &warning : engine_->warnings()) {
      RCLCPP_WARN(get_logger(), "%s", warning.c_str());
    }

    event_timer_ = create_wall_timer(50ms, std::bind(&RkllmNode::on_timer, this));
    RCLCPP_INFO(get_logger(), "rkllm_node ready");
  }

private:
  void on_user_text(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string text = trim(msg->data);
    if (text.empty()) {
      return;
    }

    if (generation_in_flight_) {
      if (busy_mode_ == "queue1") {
        if (!pending_user_text_) {
          pending_user_text_ = text;
        } else {
          publish_chat_reply("Сейчас занят, попробуйте позже.");
        }
      } else {
        publish_chat_reply("Сейчас занят, попробуйте позже.");
      }
      return;
    }

    start_generation(text);
  }

  void on_problem_pddl(const std_msgs::msg::String::SharedPtr msg)
  {
    problem_text_ = msg->data;
    problem_updated_ = now();
    have_problem_ = true;
  }

  void on_clear_cache(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string error;
    if (engine_->clear_kv_cache(&error)) {
      response->success = true;
      response->message = "cache cleared";
    } else {
      response->success = false;
      response->message = error;
    }
  }

  void start_generation(const std::string &text)
  {
    current_generation_.clear();
    last_user_text_ = text;
    generation_in_flight_ = true;

    std::string prompt = build_prompt(text);
    std::string error;
    if (!engine_->run_async(prompt, &error)) {
      generation_in_flight_ = false;
      publish_chat_reply("Не удалось запустить генерацию: " + error);
      try_start_pending();
    }
  }

  void enqueue_llm_event(const LlmEvent &event)
  {
    NodeEvent node_event;
    if (event.kind == LlmEvent::Kind::kPartial) {
      node_event.kind = NodeEvent::Kind::kLlmPartial;
      node_event.text = event.text;
    } else if (event.kind == LlmEvent::Kind::kFinish) {
      node_event.kind = NodeEvent::Kind::kLlmFinish;
    } else {
      node_event.kind = NodeEvent::Kind::kLlmError;
      node_event.text = event.text;
    }
    enqueue_event(node_event);
  }

  void enqueue_planner_status(const PlannerStatus &status)
  {
    NodeEvent node_event;
    node_event.kind = NodeEvent::Kind::kPlannerStatus;
    node_event.planner_status = status;
    enqueue_event(node_event);
  }

  void enqueue_event(const NodeEvent &event)
  {
    std::lock_guard<std::mutex> lock(event_mutex_);
    event_queue_.push_back(event);
  }

  void on_timer()
  {
    planner_client_->tick();
    std::deque<NodeEvent> pending;
    {
      std::lock_guard<std::mutex> lock(event_mutex_);
      pending.swap(event_queue_);
    }
    for (const auto &event : pending) {
      if (event.kind == NodeEvent::Kind::kLlmPartial) {
        handle_llm_partial(event.text);
      } else if (event.kind == NodeEvent::Kind::kLlmFinish) {
        handle_llm_finish();
      } else if (event.kind == NodeEvent::Kind::kLlmError) {
        handle_llm_error(event.text);
      } else if (event.kind == NodeEvent::Kind::kPlannerStatus) {
        handle_planner_status(event.planner_status);
      }
    }
  }

  void handle_llm_partial(const std::string &text)
  {
    current_generation_ += text;
    if (!publish_partials_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = text;
    partial_pub_->publish(msg);
  }

  void handle_llm_finish()
  {
    generation_in_flight_ = false;
    ParsedResponse parsed = parse_response(current_generation_);
    if (parsed.reply.empty()) {
      parsed.reply = "Готов.";
    }

    if (parsed.type == "command" && !parsed.goals.empty()) {
      publish_command_reply(parsed.reply, parsed.goals);
      std::string error;
      active_goal_summary_ = summarize_goals(parsed.goals);
      active_goal_handle_ = planner_client_->send_goals(parsed.goals, &error);
      if (active_goal_handle_.empty()) {
        publish_status("failed", "Не удалось отправить цель: " + error);
      } else {
        RCLCPP_INFO(get_logger(), "Planner monitoring started: %s", active_goal_summary_.c_str());
      }
    } else {
      publish_chat_reply(parsed.reply);
    }

    current_generation_.clear();
    try_start_pending();
  }

  void handle_llm_error(const std::string &error)
  {
    generation_in_flight_ = false;
    publish_chat_reply("Ошибка генерации: " + error);
    current_generation_.clear();
    try_start_pending();
  }

  void handle_planner_status(const PlannerStatus &status)
  {
    const bool feedback_enabled = publish_planner_feedback_;
    const std::string reply_base = active_goal_summary_.empty() ? "цель" : active_goal_summary_;

    if (status.state == PlannerStatus::State::kSubmitted && feedback_enabled) {
      publish_status("submitted", "Передано в планировщик: " + reply_base);
      return;
    }

    if (status.state == PlannerStatus::State::kRejected) {
      std::string detail = status.detail.empty() ? "отклонено" : status.detail;
      publish_status("failed", "Не получилось: " + detail);
      active_goal_handle_.clear();
      active_goal_summary_.clear();
      return;
    }

    if (status.state == PlannerStatus::State::kFailed) {
      std::string detail = status.detail.empty() ? "ошибка выполнения" : status.detail;
      publish_status("failed", "Не получилось: " + detail);
      active_goal_handle_.clear();
      active_goal_summary_.clear();
      return;
    }

    if (status.state == PlannerStatus::State::kTimeout) {
      publish_status("failed", "Не получилось: таймаут");
      active_goal_handle_.clear();
      active_goal_summary_.clear();
      return;
    }

    if (status.state == PlannerStatus::State::kSucceeded) {
      publish_status("succeeded", "Готово: " + reply_base);
      active_goal_handle_.clear();
      active_goal_summary_.clear();
      return;
    }

    if (status.state == PlannerStatus::State::kCanceled) {
      publish_status("failed", "Не получилось: отменено");
      active_goal_handle_.clear();
      active_goal_summary_.clear();
      return;
    }

    if (status.state == PlannerStatus::State::kAccepted) {
      // Goal acceptance is NOT the execution result. The current gateway
      // reports submission/acceptance only; do not mislead the operator UI.
      publish_status("accepted", "Принято планировщиком: " + reply_base);
      active_goal_handle_.clear();
      active_goal_summary_.clear();
    }
  }

  void publish_chat_reply(const std::string &reply)
  {
    json payload;
    payload["type"] = "chat";
    payload["reply"] = reply;
    publish_json(payload);
  }

  void publish_command_reply(const std::string &reply, const std::vector<std::string> &goals)
  {
    json payload;
    payload["type"] = "command";
    payload["reply"] = reply;
    payload["goals"] = goals;
    publish_json(payload);
  }

  void publish_status(const std::string &status, const std::string &reply)
  {
    json payload;
    payload["type"] = "status";
    payload["status"] = status;
    payload["reply"] = reply;
    publish_json(payload);
  }

  void publish_json(const json &payload)
  {
    if (!validate_outgoing_payload(payload)) {
      RCLCPP_WARN(get_logger(), "Outgoing payload failed schema validation");
    }
    std_msgs::msg::String msg;
    msg.data = payload.dump();
    assistant_pub_->publish(msg);
  }

  ParsedResponse parse_response(const std::string &raw)
  {
    ParsedResponse parsed;
    parsed.raw = raw;

    std::string json_blob = extract_json(raw);
    if (!json_blob.empty()) {
      try {
        json payload = json::parse(json_blob);
        if (payload.is_object() && validate_llm_payload(payload)) {
          if (payload.contains("type")) {
            parsed.type = payload["type"].get<std::string>();
          }
          if (payload.contains("reply")) {
            parsed.reply = payload["reply"].get<std::string>();
          }
          if (payload.contains("goals")) {
            if (payload["goals"].is_array()) {
              for (const auto &item : payload["goals"]) {
                if (item.is_string()) {
                  parsed.goals.push_back(item.get<std::string>());
                }
              }
            } else if (payload["goals"].is_string()) {
              parsed.goals.push_back(payload["goals"].get<std::string>());
            }
          }
        }
      } catch (const std::exception &) {
      }
    }

    if (parsed.reply.empty()) {
      parsed.reply = trim(raw);
    }
    if (parsed.type.empty()) {
      parsed.type = parsed.goals.empty() ? "chat" : "command";
    }
    return parsed;
  }

  std::string extract_json(const std::string &raw) const
  {
    const auto start = raw.find('{');
    if (start == std::string::npos) {
      return "";
    }
    int depth = 0;
    for (size_t i = start; i < raw.size(); ++i) {
      if (raw[i] == '{') {
        ++depth;
      } else if (raw[i] == '}') {
        --depth;
        if (depth == 0) {
          return raw.substr(start, i - start + 1);
        }
      }
    }
    return "";
  }

  std::string build_prompt(const std::string &user_text)
  {
    std::ostringstream prompt;
    if (!system_prompt_.empty()) {
      prompt << "SYSTEM PROMPT:\n" << system_prompt_ << "\n\n";
    }

    prompt << "STATE:\n";
    prompt << "domain_loaded=" << (domain_text_.empty() ? "false" : "true") << "\n";
    prompt << "problem_loaded=" << (problem_text_.empty() ? "false" : "true") << "\n";
    if (!domain_text_.empty() && domain_updated_.nanoseconds() > 0) {
      prompt << "domain_age_sec=" << (now() - domain_updated_).seconds() << "\n";
    }
    if (!problem_text_.empty() && problem_updated_.nanoseconds() > 0) {
      prompt << "problem_age_sec=" << (now() - problem_updated_).seconds() << "\n";
    }
    prompt << "\n";

    if (!domain_text_.empty()) {
      prompt << "DOMAIN PDDL:\n" << compress_pddl(domain_text_, max_pddl_chars_) << "\n\n";
    }
    if (!problem_text_.empty()) {
      prompt << "PROBLEM PDDL:\n" << compress_pddl(problem_text_, max_pddl_chars_) << "\n\n";
    }

    prompt << "USER:\n" << user_text;
    return prompt.str();
  }

  std::string compress_pddl(const std::string &text, size_t max_chars) const
  {
    if (text.size() <= max_chars) {
      return text;
    }

    std::string header = text.substr(0, std::min<size_t>(800, text.size()));
    std::string objects = extract_section(text, "(:objects");
    std::string init = extract_section(text, "(:init");
    std::string goal = extract_section(text, "(:goal");
    std::string tail = tail_lines(text, 20);

    std::ostringstream out;
    out << header << "\n...\n";
    if (!objects.empty()) {
      out << objects << "\n...\n";
    }
    if (!init.empty()) {
      out << init << "\n...\n";
    }
    if (!goal.empty()) {
      out << goal << "\n...\n";
    }
    out << tail;

    std::string result = out.str();
    if (result.size() > max_chars) {
      result = result.substr(result.size() - max_chars);
    }
    return result;
  }

  std::string extract_section(const std::string &text, const std::string &tag) const
  {
    auto start = text.find(tag);
    if (start == std::string::npos) {
      return "";
    }
    auto next = text.find("\n(:", start + tag.size());
    if (next == std::string::npos) {
      next = text.size();
    }
    return text.substr(start, next - start);
  }

  std::string tail_lines(const std::string &text, size_t line_count) const
  {
    size_t lines = 0;
    size_t pos = text.size();
    while (pos > 0 && lines < line_count) {
      --pos;
      if (text[pos] == '\n') {
        ++lines;
      }
    }
    return text.substr(pos);
  }

  std::string load_domain_file(const std::string &path)
  {
    std::string resolved = resolve_path(path);
    std::ifstream in(resolved);
    if (!in) {
      RCLCPP_WARN(get_logger(), "Domain file not found: %s", resolved.c_str());
      return "";
    }
    std::ostringstream content;
    content << in.rdbuf();
    domain_updated_ = now();
    have_domain_ = true;
    return content.str();
  }

  std::string load_system_prompt(const std::string &path)
  {
    std::string resolved = resolve_path(path);
    std::ifstream in(resolved);
    if (!in) {
      RCLCPP_WARN(get_logger(), "System prompt not found: %s", resolved.c_str());
      return "";
    }
    std::ostringstream content;
    content << in.rdbuf();
    std::string raw = content.str();

    try {
      json payload = json::parse(raw);
      if (payload.is_object()) {
        std::ostringstream out;
        if (payload.contains("system")) {
          out << payload["system"].get<std::string>() << "\n";
        }
        if (payload.contains("instructions")) {
          out << "INSTRUCTIONS:\n";
          for (const auto &item : payload["instructions"]) {
            if (item.is_string()) {
              out << "- " << item.get<std::string>() << "\n";
            }
          }
        }
        if (payload.contains("output_format")) {
          out << "OUTPUT FORMAT:\n" << payload["output_format"].get<std::string>() << "\n";
        }
        return out.str();
      }
    } catch (const std::exception &) {
    }

    return raw;
  }

  void setup_validators()
  {
#if HAVE_JSON_SCHEMA
    json llm_schema = {
      {"$schema", "http://json-schema.org/draft-07/schema#"},
      {"oneOf", json::array({
        json{
          {"type", "object"},
          {"required", json::array({"type", "reply"})},
          {"properties", json{
            {"type", json{{"const", "chat"}}},
            {"reply", json{{"type", "string"}}}
          }},
          {"additionalProperties", true}
        },
        json{
          {"type", "object"},
          {"required", json::array({"type", "reply", "goals"})},
          {"properties", json{
            {"type", json{{"const", "command"}}},
            {"reply", json{{"type", "string"}}},
            {"goals", json{{"type", "array"}, {"items", json{{"type", "string"}}}}}
          }},
          {"additionalProperties", true}
        }
      })}
    };

    json outgoing_schema = {
      {"$schema", "http://json-schema.org/draft-07/schema#"},
      {"oneOf", json::array({
        json{
          {"type", "object"},
          {"required", json::array({"type", "reply"})},
          {"properties", json{
            {"type", json{{"const", "chat"}}},
            {"reply", json{{"type", "string"}}}
          }},
          {"additionalProperties", true}
        },
        json{
          {"type", "object"},
          {"required", json::array({"type", "reply", "goals"})},
          {"properties", json{
            {"type", json{{"const", "command"}}},
            {"reply", json{{"type", "string"}}},
            {"goals", json{{"type", "array"}, {"items", json{{"type", "string"}}}}}
          }},
          {"additionalProperties", true}
        },
        json{
          {"type", "object"},
          {"required", json::array({"type", "status", "reply"})},
          {"properties", json{
            {"type", json{{"const", "status"}}},
            {"status", json{{"type", "string"}}},
            {"reply", json{{"type", "string"}}}
          }},
          {"additionalProperties", true}
        }
      })}
    };

    llm_validator_.set_root_schema(llm_schema);
    outgoing_validator_.set_root_schema(outgoing_schema);
#endif
  }

  bool validate_llm_payload(const json &payload)
  {
#if HAVE_JSON_SCHEMA
    try {
      llm_validator_.validate(payload);
      return true;
    } catch (const std::exception &ex) {
      RCLCPP_WARN(get_logger(), "LLM payload schema mismatch: %s", ex.what());
      return false;
    }
#else
    if (!payload.contains("type") || !payload["type"].is_string()) {
      RCLCPP_WARN(get_logger(), "LLM payload missing type");
      return false;
    }
    if (!payload.contains("reply") || !payload["reply"].is_string()) {
      RCLCPP_WARN(get_logger(), "LLM payload missing reply");
      return false;
    }
    const std::string type = payload["type"].get<std::string>();
    if (type == "chat") {
      return true;
    }
    if (type == "command") {
      if (!payload.contains("goals")) {
        RCLCPP_WARN(get_logger(), "LLM payload missing goals");
        return false;
      }
      if (payload["goals"].is_array()) {
        for (const auto &item : payload["goals"]) {
          if (!item.is_string()) {
            RCLCPP_WARN(get_logger(), "LLM payload goals must be strings");
            return false;
          }
        }
        return true;
      }
      if (payload["goals"].is_string()) {
        return true;
      }
      RCLCPP_WARN(get_logger(), "LLM payload goals has invalid type");
      return false;
    }
    RCLCPP_WARN(get_logger(), "LLM payload type not supported: %s", type.c_str());
    return false;
#endif
  }

  bool validate_outgoing_payload(const json &payload)
  {
#if HAVE_JSON_SCHEMA
    try {
      outgoing_validator_.validate(payload);
      return true;
    } catch (const std::exception &ex) {
      RCLCPP_WARN(get_logger(), "Outgoing payload schema mismatch: %s", ex.what());
      return false;
    }
#else
    if (!payload.contains("type") || !payload["type"].is_string()) {
      return false;
    }
    if (!payload.contains("reply") || !payload["reply"].is_string()) {
      return false;
    }
    const std::string type = payload["type"].get<std::string>();
    if (type == "chat") {
      return true;
    }
    if (type == "command") {
      return payload.contains("goals");
    }
    if (type == "status") {
      return payload.contains("status") && payload["status"].is_string();
    }
    return false;
#endif
  }

  std::string resolve_path(const std::string &path)
  {
    if (path.empty()) {
      return path;
    }
    if (!path.empty() && path.front() == '/') {
      return path;
    }
    const auto share = ament_index_cpp::get_package_share_directory("ros2_rknn_llm");
    return share + "/" + path;
  }

  void try_start_pending()
  {
    if (pending_user_text_) {
      std::string next = *pending_user_text_;
      pending_user_text_.reset();
      start_generation(next);
    }
  }

  std::string summarize_goals(const std::vector<std::string> &goals) const
  {
    if (goals.empty()) {
      return "";
    }
    if (goals.size() == 1) {
      return goals.front();
    }
    std::ostringstream out;
    out << goals.front();
    out << " +" << (goals.size() - 1);
    return out.str();
  }

  static std::string trim(const std::string &input)
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

  std::string model_path_;
  std::string problem_pddl_topic_;
  std::string domain_file_;
  bool publish_partials_{true};
  int max_pddl_chars_{12000};
  int max_new_tokens_{256};
  int max_context_len_{2048};
  double top_p_{0.95};
  double temperature_{0.8};
  double repeat_penalty_{1.1};
  int keep_history_{0};
  bool skip_special_token_{true};
  std::string lora_model_path_;
  std::string lora_adapter_name_;
  double lora_scale_{1.0};
  std::string prompt_cache_path_;
  bool save_prompt_cache_{false};
  std::string system_prompt_file_;
  std::string busy_mode_;
  bool publish_planner_feedback_{false};
  double planner_timeout_sec_{0.0};
  std::string planner_goal_topic_;
  std::string planner_status_topic_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr assistant_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr partial_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr user_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr problem_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_cache_srv_;
  rclcpp::TimerBase::SharedPtr event_timer_;

  std::unique_ptr<RkllmEngine> engine_;
  std::unique_ptr<PlannerClient> planner_client_;

  std::string system_prompt_;
  std::string domain_text_;
  std::string problem_text_;
  bool have_domain_{false};
  bool have_problem_{false};
  rclcpp::Time domain_updated_;
  rclcpp::Time problem_updated_;

  std::mutex event_mutex_;
  std::deque<NodeEvent> event_queue_;

  bool generation_in_flight_{false};
  std::string current_generation_;
  std::string last_user_text_;
  std::optional<std::string> pending_user_text_;

  std::string active_goal_handle_;
  std::string active_goal_summary_;

#if HAVE_JSON_SCHEMA
  json_validator llm_validator_;
  json_validator outgoing_validator_;
#endif
};

}  // namespace ros2_rknn_llm

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ros2_rknn_llm::RkllmNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(rclcpp::get_logger("rkllm_node"), "Fatal error: %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
