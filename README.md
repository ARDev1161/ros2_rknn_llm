# ros2_rknn_llm

ROS 2 C++ node (ament_cmake + rclcpp) that wraps RKLLM runtime as an LLM orchestrator.
It reads PDDL context, responds to the user with short replies, and forwards goals to
`robot_planner` / PlanSys2.

## Build

Set the RKLLM SDK path (the directory that contains `include/rkllm.h` and the arch subdir with `librkllmrt.so`) and build from workspace root:

```bash
export RKLLM_SDK_DIR=/path/to/rknn-llm/rkllm-runtime/Linux/librkllm_api
colcon build --packages-select ros2_rknn_llm
source install/setup.bash
```

If the linker cannot find the SDK at runtime, set `LD_LIBRARY_PATH` to the arch subdir:

```bash
export LD_LIBRARY_PATH=$RKLLM_SDK_DIR/aarch64:$LD_LIBRARY_PATH
```

## Run

```bash
ros2 run ros2_rknn_llm rkllm_node --ros-args -p model_path:=/path/to/model.rkllm
```

## Launch (recommended)

Launch from the orchestrator package:

```bash
ros2 launch robot_llm_orchestrator robot_llm_orchestrator.launch.py \
  model_path:=/path/to/model.rkllm
```

This launch file also starts `robot_planner/planner.launch.py` by default (includes
PlanSys2, `pddl_aggregator`, and `goal_gateway`). Disable with:

```bash
ros2 launch robot_llm_orchestrator robot_llm_orchestrator.launch.py \
  model_path:=/path/to/model.rkllm \
  enable_robot_planner:=false
```

If your PlanSys2 bringup launch file name differs, override it or disable PlanSys2:

```bash
ros2 launch robot_llm_orchestrator robot_llm_orchestrator.launch.py \
  model_path:=/path/to/model.rkllm \
  plansys2_launch_file:=plansys2_bringup_launch.py
```

```bash
ros2 launch robot_llm_orchestrator robot_llm_orchestrator.launch.py \
  model_path:=/path/to/model.rkllm \
  enable_plansys2:=false
```

Send a user request:

```bash
ros2 topic pub /rkllm_node/user_text std_msgs/msg/String "{data: 'принеси воду на кухню'}"
```

## Topics and Services

- Sub: `~/user_text` (`std_msgs/msg/String`) — user input text
- Sub: `problem_pddl_topic` (`std_msgs/msg/String`, default `/pddl/problem`) — current `problem.pddl`
- Pub: `~/assistant_text` (`std_msgs/msg/String`) — JSON replies/status updates
- Pub: `~/partial` (`std_msgs/msg/String`) — streaming chunks (optional)
- Srv: `~/clear_cache` (`std_srvs/srv/Trigger`) — RKLLM KV cache reset

## Planner integration (robot_planner)

The node uses the `robot_planner` goal gateway:

- Publishes goals to `/robot_planner/goal`
- Monitors `/robot_planner/goal_status`

The gateway reports `submitted/accepted/rejected`. Execution status is limited to what
this topic provides; with the default `robot_planner` implementation the final status
means goal acceptance/rejection (not full execution tracking). Enable
`publish_planner_feedback=true` if you want intermediate status messages.

## Output contract

The LLM must return a one-line JSON object. The node publishes structured JSON to
`~/assistant_text`:

```json
{"type":"chat","reply":"..."}
```

```json
{"type":"command","reply":"Выполняю...","goals":["(deliver pkg1 roomA)"]}
```

Status updates are published by the node after planner feedback:

```json
{"type":"status","status":"succeeded","reply":"Готово: ..."}
```

## Parameters

- `model_path` (string, required)
- `problem_pddl_topic` (string, default `/pddl/problem`)
- `domain_file` (string, default `pddl/domain.pddl`)
- `publish_partials` (bool, default `true`)
- `max_pddl_chars` (int, default `12000`)
- `max_new_tokens` (int, default `256`)
- `max_context_len` (int, default `2048`)
- `top_p` (double, default `0.95`)
- `temperature` (double, default `0.8`)
- `repeat_penalty` (double, default `1.1`)
- `keep_history` (int, default `0`)
- `skip_special_token` (bool, default `true`)
- `lora_model_path` (string, optional)
- `lora_adapter_name` (string, default `default`)
- `lora_scale` (double, default `1.0`)
- `prompt_cache_path` (string, optional)
- `save_prompt_cache` (bool, default `false`)
- `system_prompt_file` (string, default `config/system_prompt.json`)
- `busy_mode` (string, default `reject`, options: `reject|queue1`)
- `publish_planner_feedback` (bool, default `false`)
- `planner_timeout_sec` (double, default `0.0`)
- `planner_goal_topic` (string, default `/robot_planner/goal`)
- `planner_status_topic` (string, default `/robot_planner/goal_status`)

## PDDL truncation

If `domain` or `problem` exceed `max_pddl_chars`, the node keeps the header, the
`(:objects)`, `(:init)`, `(:goal)` sections (when present), and the last lines of the
file to keep the prompt compact.
