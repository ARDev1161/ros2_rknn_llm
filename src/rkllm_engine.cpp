#include "ros2_rknn_llm/rkllm_engine.hpp"

#include <dlfcn.h>
#include <cstring>
#include <filesystem>

namespace ros2_rknn_llm {

RkllmEngine::RkllmEngine(const Config &config)
: config_(config)
{
}

RkllmEngine::~RkllmEngine()
{
  shutdown();
}

bool RkllmEngine::initialize(std::string *error)
{
  if (config_.model_path.empty()) {
    if (error) {
      *error = "model_path is empty";
    }
    return false;
  }

  RKLLMParam param = rkllm_createDefaultParam();
  param.model_path = const_cast<char *>(config_.model_path.c_str());
  param.top_k = 1;
  param.top_p = config_.top_p;
  param.temperature = config_.temperature;
  param.repeat_penalty = config_.repeat_penalty;
  param.frequency_penalty = 0.0;
  param.presence_penalty = 0.0;
  param.max_new_tokens = config_.max_new_tokens;
  param.max_context_len = config_.max_context_len;
  param.skip_special_token = config_.skip_special_token;
  param.extend_param.base_domain_id = 0;
  param.extend_param.embed_flash = 1;

  int ret = rkllm_init(&handle_, &param, &RkllmEngine::callback_bridge);
  if (ret != 0 || handle_ == nullptr) {
    if (error) {
      *error = "rkllm_init failed";
    }
    return false;
  }

  int lora_ret = try_load_lora();
  if (!config_.lora_model_path.empty()) {
    if (lora_ret == -2) {
      warnings_.push_back("rkllm_load_lora not available in SDK; skipping LoRA");
    } else if (lora_ret != 0) {
      warnings_.push_back("rkllm_load_lora failed");
    }
  }

  int cache_ret = try_load_prompt_cache();
  if (!config_.prompt_cache_path.empty()) {
    if (cache_ret == -2) {
      warnings_.push_back("rkllm_load_prompt_cache not available in SDK; skipping cache load");
    } else if (cache_ret == -1) {
      warnings_.push_back("prompt_cache_path not found; skipping cache load");
    } else if (cache_ret != 0) {
      warnings_.push_back("rkllm_load_prompt_cache failed");
    }
  }
  return true;
}

void RkllmEngine::shutdown()
{
  if (worker_.joinable()) {
    worker_.join();
  }
  if (handle_) {
    LLMHandle tmp = handle_;
    handle_ = nullptr;
    rkllm_destroy(tmp);
  }
}

void RkllmEngine::set_event_callback(EventCallback cb)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  event_cb_ = std::move(cb);
}

bool RkllmEngine::run_async(const std::string &prompt, std::string *error)
{
  if (!handle_) {
    if (error) {
      *error = "rkllm not initialized";
    }
    return false;
  }
  if (running_.exchange(true)) {
    if (error) {
      *error = "rkllm already running";
    }
    return false;
  }

  if (worker_.joinable()) {
    worker_.join();
  }

  worker_ = std::thread([this, prompt]() {
    RKLLMInput input;
    std::memset(&input, 0, sizeof(RKLLMInput));
    input.input_type = RKLLM_INPUT_PROMPT;
    input.role = const_cast<char *>("user");
    input.prompt_input = const_cast<char *>(prompt.c_str());

    RKLLMInferParam infer_params;
    std::memset(&infer_params, 0, sizeof(RKLLMInferParam));
    infer_params.mode = RKLLM_INFER_GENERATE;
    infer_params.keep_history = config_.keep_history;

    RKLLMLoraParam lora_params;
    if (lora_loaded_) {
      std::memset(&lora_params, 0, sizeof(RKLLMLoraParam));
      lora_params.lora_adapter_name = const_cast<char *>(config_.lora_adapter_name.c_str());
      infer_params.lora_params = &lora_params;
    }

    RKLLMPromptCacheParam prompt_cache_params;
    if (!config_.prompt_cache_path.empty() || config_.save_prompt_cache) {
      std::memset(&prompt_cache_params, 0, sizeof(RKLLMPromptCacheParam));
      prompt_cache_params.save_prompt_cache = config_.save_prompt_cache;
      prompt_cache_params.prompt_cache_path = const_cast<char *>(config_.prompt_cache_path.c_str());
      infer_params.prompt_cache_params = &prompt_cache_params;
    }

    rkllm_run(handle_, &input, &infer_params, this);
    running_.store(false);
  });

  return true;
}

bool RkllmEngine::clear_kv_cache(std::string *error)
{
  if (!handle_) {
    if (error) {
      *error = "rkllm not initialized";
    }
    return false;
  }
  int ret = rkllm_clear_kv_cache(handle_, 1, nullptr, nullptr);
  if (ret != 0) {
    if (error) {
      *error = "rkllm_clear_kv_cache failed";
    }
    return false;
  }
  return true;
}

bool RkllmEngine::is_running() const
{
  return running_.load();
}

const std::vector<std::string> &RkllmEngine::warnings() const
{
  return warnings_;
}

int RkllmEngine::callback_bridge(RKLLMResult *result, void *userdata, LLMCallState state)
{
  auto *engine = static_cast<RkllmEngine *>(userdata);
  if (engine) {
    engine->handle_callback(result, state);
  }
  return 0;
}

void RkllmEngine::handle_callback(RKLLMResult *result, LLMCallState state)
{
  EventCallback cb;
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    cb = event_cb_;
  }
  if (!cb) {
    return;
  }

  if (state == RKLLM_RUN_NORMAL && result && result->text) {
    LlmEvent event;
    event.kind = LlmEvent::Kind::kPartial;
    event.text = result->text;
    cb(event);
  } else if (state == RKLLM_RUN_FINISH) {
    LlmEvent event;
    event.kind = LlmEvent::Kind::kFinish;
    cb(event);
  } else if (state == RKLLM_RUN_ERROR) {
    LlmEvent event;
    event.kind = LlmEvent::Kind::kError;
    event.text = "rkllm_run error";
    cb(event);
  }
}

int RkllmEngine::try_load_lora()
{
  if (config_.lora_model_path.empty()) {
    return 0;
  }

  using LoadLoraFn = int (*)(LLMHandle, RKLLMLoraAdapter *);
  void *symbol = dlsym(RTLD_DEFAULT, "rkllm_load_lora");
  if (!symbol) {
    return -2;
  }

  RKLLMLoraAdapter adapter;
  std::memset(&adapter, 0, sizeof(RKLLMLoraAdapter));
  adapter.lora_adapter_path = const_cast<char *>(config_.lora_model_path.c_str());
  adapter.lora_adapter_name = const_cast<char *>(config_.lora_adapter_name.c_str());
  adapter.scale = config_.lora_scale;

  auto fn = reinterpret_cast<LoadLoraFn>(symbol);
  int ret = fn(handle_, &adapter);
  if (ret == 0) {
    lora_loaded_ = true;
  }
  return ret;
}

int RkllmEngine::try_load_prompt_cache()
{
  if (config_.prompt_cache_path.empty()) {
    return 0;
  }
  if (!std::filesystem::exists(config_.prompt_cache_path)) {
    return -1;
  }

  using LoadPromptCacheFn = int (*)(LLMHandle, const char *);
  void *symbol = dlsym(RTLD_DEFAULT, "rkllm_load_prompt_cache");
  if (!symbol) {
    return -2;
  }

  auto fn = reinterpret_cast<LoadPromptCacheFn>(symbol);
  return fn(handle_, config_.prompt_cache_path.c_str());
}

}  // namespace ros2_rknn_llm
