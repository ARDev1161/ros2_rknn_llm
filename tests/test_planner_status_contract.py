"""Regression guards for planner handoff status semantics.

The hardware-specific RKLLM SDK is unavailable on x86 GitHub runners; these
checks intentionally validate the source-level output contract, not a ROS2 E2E
execution test. A real executor-result test is required before P4 completion.
"""
import pathlib
import re
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


def branch_body(source: str, state: str) -> str:
    pattern = r"if\s*\(status\.state\s*==\s*PlannerStatus::State::" + re.escape(state) + r"\)\s*\{"
    match = re.search(pattern, source)
    if match is None:
        raise AssertionError(f"missing planner state handler: {state}")
    start = match.end()
    level = 1
    index = start
    while level and index < len(source):
        if source[index] == "{":
            level += 1
        elif source[index] == "}":
            level -= 1
        index += 1
    if level:
        raise AssertionError("unbalanced source-level state handler")
    return source[start:index - 1]


class StatusContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (ROOT / "src/ros2_rknn_llm_node.cpp").read_text(encoding="utf-8")
        cls.client = (ROOT / "src/planner_client.cpp").read_text(encoding="utf-8")

    def test_accepted_does_not_claim_execution_completed(self):
        body = branch_body(self.source, "kAccepted")
        self.assertIn('publish_status("accepted"', body)
        self.assertNotIn('publish_status("succeeded"', body)
        self.assertNotIn('publish_status("failed"', body)

    def test_only_real_succeeded_state_reports_success(self):
        body = branch_body(self.source, "kSucceeded")
        self.assertIn('publish_status("succeeded"', body)
        self.assertNotIn('publish_status("accepted"', body)

    def test_rejection_remains_a_failure(self):
        body = branch_body(self.source, "kRejected")
        self.assertIn('publish_status("failed"', body)

    def test_gateway_status_parser_distinguishes_states(self):
        self.assertRegex(self.client, r'lower\s*==\s*"accepted"')
        self.assertIn("PlannerStatus::State::kAccepted", self.client)
        self.assertRegex(self.client, r'lower\s*==\s*"succeeded"')
        self.assertIn("PlannerStatus::State::kSucceeded", self.client)

    def test_json_output_status_uses_status_field(self):
        self.assertIn('payload["type"] = "status"', self.source)
        self.assertIn('payload["status"] = status', self.source)


if __name__ == "__main__":
    unittest.main()
