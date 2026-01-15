import json
import shlex
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass
class LLMResponse:
    goal: str
    reply: str
    raw: str


class LLMBackend:
    def generate(self, prompt: str) -> str:
        raise NotImplementedError


class MockBackend(LLMBackend):
    def generate(self, prompt: str) -> str:
        _ = prompt
        return json.dumps(
            {
                "goal": "(and (at robot1 kitchen))",
                "reply": "Goal accepted.",
            }
        )


class CliBackend(LLMBackend):
    def __init__(self, command: str):
        self.command = command

    def generate(self, prompt: str) -> str:
        args = shlex.split(self.command)
        completed = subprocess.run(
            args,
            input=prompt,
            text=True,
            capture_output=True,
            check=False,
        )
        if completed.returncode != 0:
            stderr = completed.stderr.strip()
            raise RuntimeError(f"LLM CLI failed: {stderr}")
        return completed.stdout.strip()


class RknnBackend(LLMBackend):
    def __init__(self, model_path: str):
        self.model_path = model_path
        try:
            import rknn_llm  # type: ignore
        except Exception as exc:
            raise RuntimeError("rknn_llm python API not available") from exc
        self.rknn_llm = rknn_llm
        self.model = None

    def load(self):
        if self.model is None:
            self.model = self.rknn_llm.Model(self.model_path)

    def generate(self, prompt: str) -> str:
        self.load()
        return self.model.generate(prompt)


class Ros2RknnLlm(Node):
    def __init__(self):
        super().__init__("ros2_rknn_llm")

        self.backend_type = self.declare_parameter("backend", "mock").value
        self.model_path = self.declare_parameter("model_path", "").value
        self.cli_command = self.declare_parameter("cli_command", "").value
        self.system_prompt_path = self.declare_parameter(
            "system_prompt_path", "config/system_prompt.json"
        ).value
        self.asr_topic = self.declare_parameter("asr_topic", "/llm/asr_in").value
        self.tts_topic = self.declare_parameter("tts_topic", "/llm/tts_out").value
        self.goal_topic = self.declare_parameter("goal_topic", "/robot_planner/goal").value
        self.domain_service = self.declare_parameter("domain_service", "/pddl/get_domain").value
        self.problem_service = self.declare_parameter(
            "problem_service", "/pddl/get_problem"
        ).value
        self.request_timeout = self.declare_parameter("request_timeout_sec", 2.0).value
        self.max_context_chars = int(
            self.declare_parameter("max_context_chars", 12000).value
        )
        self.busy = False

        self.domain_client = self.create_client(Trigger, self.domain_service)
        self.problem_client = self.create_client(Trigger, self.problem_service)

        self.asr_sub = self.create_subscription(String, self.asr_topic, self.on_asr, 10)
        self.tts_pub = self.create_publisher(String, self.tts_topic, 10)
        self.goal_pub = self.create_publisher(String, self.goal_topic, 10)

        self.backend = self.create_backend()
        self.system_prompt = self.load_system_prompt()

    def create_backend(self) -> LLMBackend:
        backend = self.backend_type.lower().strip()
        if backend == "mock":
            return MockBackend()
        if backend == "cli":
            if not self.cli_command:
                raise RuntimeError("cli_command is required for CLI backend.")
            return CliBackend(self.cli_command)
        if backend == "rknn":
            if not self.model_path:
                raise RuntimeError("model_path is required for rknn backend.")
            return RknnBackend(self.model_path)
        raise RuntimeError(f"Unknown backend: {self.backend_type}")

    def load_system_prompt(self) -> dict:
        path = Path(self.system_prompt_path)
        if not path.is_absolute():
            share_dir = Path(get_package_share_directory("ros2_rknn_llm"))
            path = share_dir / self.system_prompt_path
        if not path.exists():
            self.get_logger().warn(f"system_prompt_path not found: {path}")
            return {}
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse system prompt JSON: {exc}")
            return {}

    def request_pddl(self, client: rclpy.client.Client) -> str:
        if not client.wait_for_service(timeout_sec=float(self.request_timeout)):
            self.get_logger().warn(f"Service not available: {client.srv_name}")
            return ""
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=float(self.request_timeout))
        if not future.done():
            self.get_logger().warn(f"Service timeout: {client.srv_name}")
            return ""
        result = future.result()
        if not result or not result.success:
            return ""
        return result.message

    def build_prompt(self, user_text: str, domain: str, problem: str) -> str:
        system = self.system_prompt.get("system", "")
        instructions = self.system_prompt.get("instructions", [])
        output_format = self.system_prompt.get("output_format", "")
        parts = []
        if system:
            parts.append(f"SYSTEM:\n{system}")
        if instructions:
            parts.append("INSTRUCTIONS:\n" + "\n".join(f"- {item}" for item in instructions))
        if output_format:
            parts.append(f"OUTPUT FORMAT:\n{output_format}")
        if domain:
            parts.append(f"DOMAIN PDDL:\n{domain}")
        if problem:
            parts.append(f"PROBLEM PDDL:\n{problem}")
        parts.append(f"USER REQUEST:\n{user_text}")
        prompt = "\n\n".join(parts)
        if len(prompt) > self.max_context_chars:
            prompt = prompt[-self.max_context_chars :]
        return prompt

    def parse_response(self, raw: str) -> LLMResponse:
        goal = ""
        reply = ""
        try:
            payload = json.loads(raw)
            if isinstance(payload, dict):
                goal = str(payload.get("goal", "")).strip()
                reply = str(payload.get("reply", "")).strip()
        except Exception:
            pass
        if not goal:
            goal = raw.strip()
        return LLMResponse(goal=goal, reply=reply, raw=raw)

    def on_asr(self, msg: String) -> None:
        if self.busy:
            self.get_logger().warn("LLM busy; dropping request.")
            return
        text = msg.data.strip()
        if not text:
            return
        self.busy = True
        try:
            domain = self.request_pddl(self.domain_client)
            problem = self.request_pddl(self.problem_client)
            prompt = self.build_prompt(text, domain, problem)
            raw = self.backend.generate(prompt)
            result = self.parse_response(raw)
            if result.goal:
                goal_msg = String()
                goal_msg.data = result.goal
                self.goal_pub.publish(goal_msg)
                self.get_logger().info(f"Published goal: {result.goal}")
            if result.reply:
                tts_msg = String()
                tts_msg.data = result.reply
                self.tts_pub.publish(tts_msg)
        except Exception as exc:
            self.get_logger().error(f"LLM request failed: {exc}")
        finally:
            self.busy = False


def main() -> None:
    rclpy.init()
    node = Ros2RknnLlm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
