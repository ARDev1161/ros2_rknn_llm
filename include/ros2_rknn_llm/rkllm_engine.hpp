#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rkllm.h"

namespace ros2_rknn_llm {

struct LlmEvent {
  enum class Kind { kPartial, kFinish, kError };
  Kind kind{Kind::kPartial};
  std::string text;
};

class RkllmEngine {
public:
  struct Config {
    std::string model_path;
    int max_new_tokens{256};
    int max_context_len{2048};
    double top_p{0.95};
    double temperature{0.8};
    double repeat_penalty{1.1};
    int keep_history{0};
    bool skip_special_token{true};
    std::string lora_model_path;
    std::string lora_adapter_name{"default"};
    double lora_scale{1.0};
    std::string prompt_cache_path;
    bool save_prompt_cache{false};
  };

  using EventCallback = std::function<void(const LlmEvent &)>;

  explicit RkllmEngine(const Config &config);
  ~RkllmEngine();

  bool initialize(std::string *error);
  void shutdown();

  void set_event_callback(EventCallback cb);
  bool run_async(const std::string &prompt, std::string *error);
  bool clear_kv_cache(std::string *error);
  bool is_running() const;
  const std::vector<std::string> &warnings() const;

private:
  static int callback_bridge(RKLLMResult *result, void *userdata, LLMCallState state);
  void handle_callback(RKLLMResult *result, LLMCallState state);

  int try_load_lora();
  int try_load_prompt_cache();

  Config config_;
  LLMHandle handle_{nullptr};
  std::atomic<bool> running_{false};
  std::thread worker_;
  std::mutex callback_mutex_;
  EventCallback event_cb_;
  bool lora_loaded_{false};
  std::vector<std::string> warnings_;
};

}  // namespace ros2_rknn_llm
