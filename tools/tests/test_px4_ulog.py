#!/usr/bin/env python3

import importlib.util
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace


TEST_DIRECTORY = Path(__file__).resolve().parent
SCRIPT_PATH = TEST_DIRECTORY.parent / "px4_ulog.py"


def load_module():
    spec = importlib.util.spec_from_file_location("px4_ulog", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Px4UlogTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_module()

    def entry(self, log_id, size=1024, utc_seconds=0):
        return self.module.LogInfo(log_id=log_id, size=size, utc_seconds=utc_seconds)

    def test_latest_log_is_selected_by_id_not_arrival_order(self):
        entries = [self.entry(8), self.entry(11), self.entry(9)]

        selected = self.module.select_log(entries, latest=True, requested_id=None)

        self.assertEqual(11, selected.log_id)

    def test_requested_log_id_must_exist(self):
        entries = [self.entry(3), self.entry(4)]

        with self.assertRaisesRegex(self.module.Px4LogError, "log id 7"):
            self.module.select_log(entries, latest=False, requested_id=7)

    def test_output_name_uses_px4_utc_time_when_available(self):
        entry = self.entry(54, utc_seconds=1_790_177_588)

        name = self.module.output_filename(entry)

        self.assertEqual("flight_0054_20260923_153308.ulg", name)

    def test_output_name_is_stable_when_px4_has_no_utc_time(self):
        self.assertEqual("flight_0054.ulg", self.module.output_filename(self.entry(54)))

    def test_coverage_merges_overlap_and_reports_bounded_missing_ranges(self):
        coverage = self.module.Coverage(total_size=100)
        coverage.add(0, 20)
        coverage.add(40, 60)
        coverage.add(15, 45)

        self.assertEqual(60, coverage.covered_bytes)
        self.assertEqual([(60, 25), (85, 15)], coverage.missing_ranges(max_count=25))

    def test_coverage_round_trip_preserves_resume_state(self):
        coverage = self.module.Coverage(total_size=100)
        coverage.add(0, 30)
        coverage.add(50, 80)

        restored = self.module.Coverage.from_intervals(100, coverage.intervals)

        self.assertEqual([(0, 30), (50, 80)], restored.intervals)
        self.assertEqual([(30, 20), (80, 20)], restored.missing_ranges(max_count=100))

    def test_complete_existing_ulog_is_reused_only_when_size_and_magic_match(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "flight_0007.ulg"
            output.write_bytes(self.module.ULOG_MAGIC + b"payload")
            entry = self.entry(7, size=output.stat().st_size)

            self.assertTrue(self.module.is_complete_ulog(output, entry.size))

            output.write_bytes(b"broken!" + b"payload")
            self.assertFalse(self.module.is_complete_ulog(output, output.stat().st_size))

    def test_no_cli_arguments_defaults_to_latest_download(self):
        self.assertEqual(
            ["download", "--latest"],
            self.module.normalize_argv([]),
        )

    def test_download_without_selector_defaults_to_latest(self):
        self.assertEqual(
            ["download", "--latest", "--timeout", "30"],
            self.module.normalize_argv(["download", "--timeout", "30"]),
        )

    def test_log_list_request_is_retried_when_first_request_gets_no_entries(self):
        callbacks = []
        request_count = 0

        class Subscriber:
            def unregister(self):
                pass

        def request_list(_start, _end):
            nonlocal request_count
            request_count += 1
            if request_count == 2:
                callbacks[0](
                    SimpleNamespace(
                        id=12,
                        size=3456,
                        num_logs=1,
                        time_utc=SimpleNamespace(to_sec=lambda: 1_790_177_588),
                    )
                )
            return SimpleNamespace(success=True)

        fake_rospy = SimpleNamespace(
            Subscriber=lambda _topic, _type, callback, queue_size: (
                callbacks.append(callback) or Subscriber()
            ),
            wait_for_service=lambda _name, timeout: None,
            ServiceProxy=lambda _name, _type: request_list,
            is_shutdown=lambda: False,
            sleep=time.sleep,
        )

        entries = self.module.collect_log_entries(
            fake_rospy,
            LogEntry=object,
            LogRequestList=object,
            namespace="/mavros/log_transfer/raw",
            timeout=0.3,
            retry_interval=0.02,
        )

        self.assertEqual([self.entry(12, 3456, 1_790_177_588)], entries)
        self.assertEqual(2, request_count)

    def test_unavailable_mavros_log_service_reports_a_tool_error(self):
        class Subscriber:
            def unregister(self):
                pass

        fake_rospy = SimpleNamespace(
            Subscriber=lambda *_args, **_kwargs: Subscriber(),
            wait_for_service=lambda _name, timeout: (_ for _ in ()).throw(
                RuntimeError("service unavailable")
            ),
            ServiceProxy=lambda *_args: None,
            is_shutdown=lambda: False,
            sleep=time.sleep,
        )

        with self.assertRaisesRegex(
            self.module.Px4LogError, "MAVROS log list service unavailable"
        ):
            self.module.collect_log_entries(
                fake_rospy,
                LogEntry=object,
                LogRequestList=object,
                namespace="/mavros/log_transfer/raw",
                timeout=0.1,
            )


if __name__ == "__main__":
    unittest.main()
