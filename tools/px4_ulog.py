#!/usr/bin/env python3

"""List and download PX4 ULog files through MAVROS LOG_TRANSFER.

The tool never erases logs on the flight controller. Downloads are written to
``<workspace>/log/px4`` by default. An incomplete transfer is kept as ``.part``
plus a compact interval map so a later invocation can resume it safely.
"""

import argparse
import datetime
import json
import os
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Iterable, List, NamedTuple, Optional, Sequence, Tuple


ULOG_MAGIC = b"ULog\x01\x12\x35"
DEFAULT_NAMESPACE = "/mavros/log_transfer/raw"
DEFAULT_OUTPUT_DIRECTORY = Path(__file__).resolve().parent.parent / "log" / "px4"
STATE_VERSION = 1


class Px4LogError(RuntimeError):
    """Expected command or transfer failure with a concise user-facing message."""


class LogInfo(NamedTuple):
    log_id: int
    size: int
    utc_seconds: int = 0


class Coverage:
    """Compact set of covered byte intervals using half-open ranges."""

    def __init__(self, total_size: int):
        if total_size <= 0:
            raise ValueError("total_size must be positive")
        self.total_size = total_size
        self.intervals: List[Tuple[int, int]] = []
        self.covered_bytes = 0

    @classmethod
    def from_intervals(
        cls, total_size: int, intervals: Iterable[Sequence[int]]
    ) -> "Coverage":
        coverage = cls(total_size)
        for interval in intervals:
            if len(interval) != 2:
                raise ValueError("coverage interval must contain start and end")
            coverage.add(int(interval[0]), int(interval[1]))
        return coverage

    def add(self, start: int, end: int) -> int:
        if start < 0 or end <= start or end > self.total_size:
            raise ValueError(
                "invalid coverage interval [{}, {}) for {} bytes".format(
                    start, end, self.total_size
                )
            )

        previous_covered = self.covered_bytes
        merged_start = start
        merged_end = end
        output: List[Tuple[int, int]] = []
        inserted = False

        for current_start, current_end in self.intervals:
            if current_end < merged_start:
                output.append((current_start, current_end))
            elif merged_end < current_start:
                if not inserted:
                    output.append((merged_start, merged_end))
                    inserted = True
                output.append((current_start, current_end))
            else:
                merged_start = min(merged_start, current_start)
                merged_end = max(merged_end, current_end)

        if not inserted:
            output.append((merged_start, merged_end))

        self.intervals = output
        self.covered_bytes = sum(end - start for start, end in output)
        return self.covered_bytes - previous_covered

    @property
    def complete(self) -> bool:
        return self.covered_bytes == self.total_size

    def missing_ranges(self, max_count: int) -> List[Tuple[int, int]]:
        if max_count <= 0:
            raise ValueError("max_count must be positive")

        output: List[Tuple[int, int]] = []
        cursor = 0
        for start, end in self.intervals:
            self._append_gap(output, cursor, start, max_count)
            cursor = end
        self._append_gap(output, cursor, self.total_size, max_count)
        return output

    @staticmethod
    def _append_gap(
        output: List[Tuple[int, int]], start: int, end: int, max_count: int
    ) -> None:
        while start < end:
            count = min(max_count, end - start)
            output.append((start, count))
            start += count


def select_log(
    entries: Sequence[LogInfo], latest: bool, requested_id: Optional[int]
) -> LogInfo:
    if not entries:
        raise Px4LogError("PX4 returned no log entries")
    if requested_id is not None:
        for entry in entries:
            if entry.log_id == requested_id:
                return entry
        raise Px4LogError("log id {} was not reported by PX4".format(requested_id))
    if not latest:
        raise Px4LogError("select --latest or provide --id")
    return max(entries, key=lambda entry: entry.log_id)


def output_filename(entry: LogInfo) -> str:
    if entry.utc_seconds > 0:
        timestamp = datetime.datetime.fromtimestamp(
            entry.utc_seconds, tz=datetime.timezone.utc
        ).strftime("%Y%m%d_%H%M%S")
        return "flight_{:04d}_{}.ulg".format(entry.log_id, timestamp)
    return "flight_{:04d}.ulg".format(entry.log_id)


def is_complete_ulog(path: Path, expected_size: int) -> bool:
    if not path.is_file() or path.stat().st_size != expected_size:
        return False
    with path.open("rb") as stream:
        return stream.read(len(ULOG_MAGIC)) == ULOG_MAGIC


def normalize_argv(argv: Sequence[str]) -> List[str]:
    normalized = list(argv)
    if not normalized:
        return ["download", "--latest"]
    if normalized[0] == "download" and not any(
        argument == "--latest" or argument == "--id" or argument.startswith("--id=")
        for argument in normalized[1:]
    ):
        normalized.insert(1, "--latest")
    return normalized


def _human_size(size: int) -> str:
    value = float(size)
    for unit in ("B", "KiB", "MiB", "GiB"):
        if value < 1024.0 or unit == "GiB":
            return "{:.1f} {}".format(value, unit)
        value /= 1024.0
    return "{} B".format(size)


def _utc_text(seconds: int) -> str:
    if seconds <= 0:
        return "unknown"
    return datetime.datetime.fromtimestamp(
        seconds, tz=datetime.timezone.utc
    ).strftime("%Y-%m-%d %H:%M:%S UTC")


def _service(namespace: str, suffix: str) -> str:
    return "{}/{}".format(namespace.rstrip("/"), suffix)


def _log_info_from_message(message) -> LogInfo:
    try:
        utc_seconds = int(message.time_utc.to_sec())
    except (AttributeError, TypeError, ValueError):
        utc_seconds = 0
    return LogInfo(int(message.id), int(message.size), utc_seconds)


def _wait_for_service(rospy, service_name: str, timeout: float, description: str) -> None:
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
    except Exception as error:
        raise Px4LogError(
            "MAVROS {} service unavailable: {}".format(description, error)
        ) from error


def collect_log_entries(
    rospy,
    LogEntry,
    LogRequestList,
    namespace: str,
    timeout: float,
    retry_interval: float = 2.0,
) -> List[LogInfo]:
    entries: Dict[int, LogInfo] = {}
    expected_count = 0
    last_message_at = 0.0
    lock = threading.Lock()

    def callback(message):
        nonlocal expected_count, last_message_at
        with lock:
            entries[int(message.id)] = _log_info_from_message(message)
            expected_count = max(expected_count, int(message.num_logs))
            last_message_at = time.monotonic()

    topic = _service(namespace, "log_entry")
    service_name = _service(namespace, "log_request_list")
    subscriber = rospy.Subscriber(topic, LogEntry, callback, queue_size=1000)
    try:
        _wait_for_service(
            rospy, service_name, min(timeout, 10.0), "log list"
        )
        request_list = rospy.ServiceProxy(service_name, LogRequestList)
        if not request_list(0, 65535).success:
            raise Px4LogError("PX4 rejected the log list request")

        started = time.monotonic()
        last_request_at = started
        while not rospy.is_shutdown():
            now = time.monotonic()
            with lock:
                count = len(entries)
                expected = expected_count
                idle = now - last_message_at if last_message_at else 0.0
            if count and expected > 0 and count >= expected:
                break
            if count and expected == 0 and idle >= 0.75:
                break
            if now - started >= timeout:
                if count:
                    break
                raise Px4LogError(
                    "timed out waiting for PX4 log entries after {:.1f} s".format(timeout)
                )
            if (
                now - last_request_at >= retry_interval
                and (not count or idle >= retry_interval)
            ):
                if not request_list(0, 65535).success:
                    raise Px4LogError("PX4 rejected a repeated log list request")
                last_request_at = now
            rospy.sleep(0.05)
    finally:
        subscriber.unregister()

    return sorted(entries.values(), key=lambda entry: entry.log_id)


def _state_paths(output_path: Path) -> Tuple[Path, Path]:
    part_path = output_path.with_name(output_path.name + ".part")
    state_path = output_path.with_name(output_path.name + ".part.json")
    return part_path, state_path


def _save_state(state_path: Path, entry: LogInfo, coverage: Coverage) -> None:
    payload = {
        "version": STATE_VERSION,
        "log_id": entry.log_id,
        "size": entry.size,
        "intervals": coverage.intervals,
    }
    temporary_path = state_path.with_name(state_path.name + ".tmp")
    with temporary_path.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, separators=(",", ":"))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary_path), str(state_path))


def _load_or_create_state(
    entry: LogInfo, part_path: Path, state_path: Path, force: bool
) -> Coverage:
    if force:
        for path in (part_path, state_path):
            if path.exists():
                path.unlink()

    if part_path.is_file() and state_path.is_file():
        try:
            payload = json.loads(state_path.read_text(encoding="utf-8"))
            if (
                payload.get("version") == STATE_VERSION
                and int(payload.get("log_id")) == entry.log_id
                and int(payload.get("size")) == entry.size
                and part_path.stat().st_size == entry.size
            ):
                return Coverage.from_intervals(entry.size, payload.get("intervals", []))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass

    for path in (part_path, state_path):
        if path.exists():
            path.unlink()
    with part_path.open("wb") as stream:
        stream.truncate(entry.size)
    coverage = Coverage(entry.size)
    _save_state(state_path, entry, coverage)
    return coverage


def download_log(
    rospy,
    LogData,
    LogRequestData,
    LogRequestEnd,
    namespace: str,
    entry: LogInfo,
    output_directory: Path,
    timeout: float,
    stall_timeout: float,
    retry_bytes: int,
    force: bool,
) -> Path:
    output_directory.mkdir(parents=True, exist_ok=True)
    output_path = output_directory / output_filename(entry)
    part_path, state_path = _state_paths(output_path)

    if output_path.exists():
        if is_complete_ulog(output_path, entry.size) and not force:
            print("SKIP: complete log already exists: {}".format(output_path), flush=True)
            return output_path
        if not force:
            raise Px4LogError(
                "output exists but is incomplete or invalid: {}; use --force to replace it".format(
                    output_path
                )
            )
        output_path.unlink()

    coverage = _load_or_create_state(entry, part_path, state_path, force)
    lock = threading.Lock()
    last_progress_at = time.monotonic()
    transfer_error: List[str] = []

    with part_path.open("r+b", buffering=0) as part_stream:

        def callback(message):
            nonlocal last_progress_at
            if int(message.id) != entry.log_id:
                return
            offset = int(message.offset)
            chunk = bytes(message.data)
            end = min(offset + len(chunk), entry.size)
            if offset < 0 or offset >= entry.size or end <= offset:
                return
            chunk = chunk[: end - offset]
            try:
                with lock:
                    part_stream.seek(offset)
                    part_stream.write(chunk)
                    newly_covered = coverage.add(offset, end)
                    if newly_covered:
                        last_progress_at = time.monotonic()
            except (OSError, ValueError) as error:
                with lock:
                    transfer_error.append(str(error))

        data_topic = _service(namespace, "log_data")
        data_service = _service(namespace, "log_request_data")
        end_service = _service(namespace, "log_request_end")
        subscriber = rospy.Subscriber(data_topic, LogData, callback, queue_size=20000)
        request_end = None
        try:
            _wait_for_service(rospy, data_service, 10.0, "log data")
            request_data = rospy.ServiceProxy(data_service, LogRequestData)
            request_end = rospy.ServiceProxy(end_service, LogRequestEnd)

            with lock:
                missing = coverage.missing_ranges(max(entry.size, 1))
            if missing:
                initial_offset = missing[0][0]
                initial_count = entry.size - initial_offset
                if not request_data(entry.log_id, initial_offset, initial_count).success:
                    raise Px4LogError("PX4 rejected the initial log data request")

            started = time.monotonic()
            last_report_at = 0.0
            last_retry_at = 0.0
            last_state_save_at = 0.0
            while not rospy.is_shutdown():
                now = time.monotonic()
                with lock:
                    count = coverage.covered_bytes
                    complete = coverage.complete
                    stalled_for = now - last_progress_at
                    error_text = transfer_error[0] if transfer_error else ""
                if error_text:
                    raise Px4LogError("failed to write partial log: {}".format(error_text))
                if complete:
                    break
                if now - started >= timeout:
                    raise Px4LogError(
                        "download timed out with {}/{} bytes; rerun to resume".format(
                            count, entry.size
                        )
                    )
                if now - last_report_at >= 2.0:
                    print(
                        "progress: {:.1f}% ({}/{})".format(
                            100.0 * count / entry.size, count, entry.size
                        ),
                        flush=True,
                    )
                    last_report_at = now
                if now - last_state_save_at >= 2.0:
                    with lock:
                        _save_state(state_path, entry, coverage)
                    last_state_save_at = now
                if stalled_for >= stall_timeout and now - last_retry_at >= stall_timeout:
                    with lock:
                        ranges = coverage.missing_ranges(retry_bytes)
                    if ranges:
                        offset, count_to_request = ranges[0]
                        response = request_data(entry.log_id, offset, count_to_request)
                        if not response.success:
                            raise Px4LogError(
                                "PX4 rejected retry for bytes {}..{}".format(
                                    offset, offset + count_to_request
                                )
                            )
                        last_retry_at = now
                rospy.sleep(0.05)

            with lock:
                _save_state(state_path, entry, coverage)
            part_stream.flush()
            os.fsync(part_stream.fileno())
        finally:
            subscriber.unregister()
            if request_end is not None:
                try:
                    request_end()
                except Exception:
                    pass

    if not is_complete_ulog(part_path, entry.size):
        raise Px4LogError(
            "downloaded byte count is complete but ULog header/size validation failed; "
            "partial file kept at {}".format(part_path)
        )
    os.replace(str(part_path), str(output_path))
    if state_path.exists():
        state_path.unlink()
    print("saved {} bytes to {}".format(entry.size, output_path), flush=True)
    return output_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="List and download PX4 ULog files through an existing MAVROS node."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="list ULogs stored on PX4")
    list_parser.add_argument("--namespace", default=DEFAULT_NAMESPACE)
    list_parser.add_argument("--timeout", type=float, default=15.0)

    download_parser = subparsers.add_parser("download", help="download one PX4 ULog")
    selector = download_parser.add_mutually_exclusive_group(required=True)
    selector.add_argument("--latest", action="store_true", help="download the highest log id")
    selector.add_argument("--id", type=int, dest="log_id", help="download a specific log id")
    download_parser.add_argument("--namespace", default=DEFAULT_NAMESPACE)
    download_parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIRECTORY)
    download_parser.add_argument("--list-timeout", type=float, default=15.0)
    download_parser.add_argument("--timeout", type=float, default=300.0)
    download_parser.add_argument("--stall-timeout", type=float, default=1.0)
    download_parser.add_argument("--retry-bytes", type=int, default=45000)
    download_parser.add_argument(
        "--force", action="store_true", help="replace the exact local output and resume files"
    )
    return parser


def _validate_args(args) -> None:
    for name in ("timeout", "list_timeout", "stall_timeout"):
        if hasattr(args, name) and getattr(args, name) <= 0:
            raise Px4LogError("--{} must be positive".format(name.replace("_", "-")))
    if hasattr(args, "retry_bytes") and args.retry_bytes <= 0:
        raise Px4LogError("--retry-bytes must be positive")
    if getattr(args, "log_id", 0) is not None and getattr(args, "log_id", 0) < 0:
        raise Px4LogError("--id must be non-negative")


def main(argv: Optional[Sequence[str]] = None) -> int:
    try:
        import rospy
        from mavros_msgs.msg import LogData, LogEntry
        from mavros_msgs.srv import LogRequestData, LogRequestEnd, LogRequestList
    except ImportError as error:
        print(
            "ERROR: ROS Noetic/MAVROS Python packages are unavailable; source "
            "/opt/ros/noetic/setup.bash first ({})".format(error),
            file=sys.stderr,
        )
        return 2

    cli_argv = rospy.myargv(argv=sys.argv if argv is None else [sys.argv[0], *argv])[1:]
    parser = build_parser()
    args = parser.parse_args(normalize_argv(cli_argv))

    try:
        _validate_args(args)
        rospy.init_node("px4_ulog_tool", anonymous=True, disable_signals=True)
        list_timeout = args.timeout if args.command == "list" else args.list_timeout
        entries = collect_log_entries(
            rospy, LogEntry, LogRequestList, args.namespace, list_timeout
        )

        if args.command == "list":
            print("ID    Size        PX4 UTC")
            for entry in entries:
                print(
                    "{:4d}  {:>10s}  {}".format(
                        entry.log_id, _human_size(entry.size), _utc_text(entry.utc_seconds)
                    )
                )
            return 0

        entry = select_log(entries, args.latest, args.log_id)
        print(
            "selected log {}: {}, {}".format(
                entry.log_id, _human_size(entry.size), _utc_text(entry.utc_seconds)
            ),
            flush=True,
        )
        download_log(
            rospy,
            LogData,
            LogRequestData,
            LogRequestEnd,
            args.namespace,
            entry,
            args.output_dir.expanduser().resolve(),
            args.timeout,
            args.stall_timeout,
            args.retry_bytes,
            args.force,
        )
        return 0
    except (Px4LogError, OSError, ValueError) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
