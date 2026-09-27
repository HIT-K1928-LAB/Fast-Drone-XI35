#!/usr/bin/env python3

"""Merge FAST-LIVO2 binary PCD slices under each tmux session log."""

import argparse
import datetime
import os
import re
import shutil
import sys
import tempfile
from decimal import Decimal
from pathlib import Path
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple


SLICE_NAME_RE = re.compile(r"^[0-9]+(?:\.[0-9]+)?\.pcd$")
SESSION_TIME_RE = re.compile(r"^([0-9]{4}-[0-9]{2}-[0-9]{2}_[0-9]{2}-[0-9]{2}-[0-9]{2})(?:_|$)")
COPY_CHUNK_BYTES = 8 * 1024 * 1024
FREE_SPACE_RESERVE_BYTES = 64 * 1024 * 1024


class PcdError(RuntimeError):
    pass


class PcdMetadata(NamedTuple):
    path: Path
    header_lines: Tuple[str, ...]
    point_count: int
    point_step: int
    data_offset: int
    payload_bytes: int
    schema: Tuple[object, ...]


def _parse_header_value(values: Dict[str, List[str]], key: str, path: Path) -> List[str]:
    if key not in values or not values[key]:
        raise PcdError("{}: missing {} in PCD header".format(path, key))
    return values[key]


def inspect_binary_pcd(path: Path) -> PcdMetadata:
    header_lines = []
    values = {}
    data_offset = 0

    with path.open("rb") as stream:
        for _ in range(256):
            raw_line = stream.readline(65537)
            if not raw_line:
                raise PcdError("{}: PCD header has no DATA line".format(path))
            if len(raw_line) > 65536:
                raise PcdError("{}: PCD header line is too long".format(path))
            try:
                line = raw_line.decode("ascii").rstrip("\r\n")
            except UnicodeDecodeError as error:
                raise PcdError("{}: PCD header is not ASCII".format(path)) from error
            header_lines.append(line)
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            tokens = stripped.split()
            key = tokens[0].upper()
            values[key] = tokens[1:]
            if key == "DATA":
                data_offset = stream.tell()
                break
        else:
            raise PcdError("{}: PCD header exceeds 256 lines".format(path))

    data_kind = _parse_header_value(values, "DATA", path)[0].lower()
    if data_kind != "binary":
        raise PcdError("{}: only DATA binary PCD files are supported, found {}".format(path, data_kind))

    fields = tuple(_parse_header_value(values, "FIELDS", path))
    sizes = tuple(int(value) for value in _parse_header_value(values, "SIZE", path))
    types = tuple(_parse_header_value(values, "TYPE", path))
    counts = tuple(int(value) for value in values.get("COUNT", ["1"] * len(fields)))
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise PcdError("{}: inconsistent FIELDS/SIZE/TYPE/COUNT schema".format(path))
    if any(size <= 0 for size in sizes) or any(count <= 0 for count in counts):
        raise PcdError("{}: SIZE and COUNT must be positive".format(path))

    point_count = int(_parse_header_value(values, "POINTS", path)[0])
    width = int(_parse_header_value(values, "WIDTH", path)[0])
    height = int(_parse_header_value(values, "HEIGHT", path)[0])
    if point_count < 0 or width < 0 or height != 1 or width != point_count:
        raise PcdError("{}: expected an unorganized cloud with WIDTH == POINTS and HEIGHT == 1".format(path))

    point_step = sum(size * count for size, count in zip(sizes, counts))
    payload_bytes = point_count * point_step
    actual_payload_bytes = path.stat().st_size - data_offset
    if actual_payload_bytes != payload_bytes:
        raise PcdError(
            "{}: payload size mismatch (expected {}, found {})".format(
                path, payload_bytes, actual_payload_bytes))

    schema = (
        values.get("VERSION", []),
        fields,
        sizes,
        types,
        counts,
        tuple(values.get("VIEWPOINT", [])),
        data_kind,
    )
    return PcdMetadata(
        path=path,
        header_lines=tuple(header_lines),
        point_count=point_count,
        point_step=point_step,
        data_offset=data_offset,
        payload_bytes=payload_bytes,
        schema=schema,
    )


def _merged_header(first: PcdMetadata, total_points: int) -> bytes:
    output_lines = []
    for line in first.header_lines:
        stripped = line.strip()
        key = stripped.split(maxsplit=1)[0].upper() if stripped and not stripped.startswith("#") else ""
        if key == "WIDTH":
            output_lines.append("WIDTH {}".format(total_points))
        elif key == "HEIGHT":
            output_lines.append("HEIGHT 1")
        elif key == "POINTS":
            output_lines.append("POINTS {}".format(total_points))
        else:
            output_lines.append(line)
    return ("\n".join(output_lines) + "\n").encode("ascii")


def _session_output_name(session_dir: Path) -> str:
    match = SESSION_TIME_RE.match(session_dir.name)
    if match:
        timestamp = match.group(1)
    else:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return "{}_raw.pcd".format(timestamp)


def _slice_sort_key(path: Path) -> Decimal:
    return Decimal(path.stem)


def merge_pcd_directory(pcd_dir: Path, dry_run: bool = False) -> str:
    merged_outputs = sorted(pcd_dir.glob("*_raw.pcd"))
    slice_paths = sorted(
        (path for path in pcd_dir.iterdir() if path.is_file() and SLICE_NAME_RE.fullmatch(path.name)),
        key=_slice_sort_key,
    )

    if merged_outputs:
        print("SKIP {}: already merged as {}".format(pcd_dir, merged_outputs[0].name))
        return "skipped"
    if not slice_paths:
        return "empty"

    metadata = []
    corrupt_count = 0
    for path in slice_paths:
        try:
            metadata.append(inspect_binary_pcd(path))
        except (OSError, ValueError, PcdError) as error:
            corrupt_count += 1
            print("SKIP CORRUPT {}: {}".format(path, error))

    if not metadata:
        raise PcdError("{}: no valid PCD slices; preserved {} corrupt slices".format(
            pcd_dir, corrupt_count))

    first = metadata[0]
    for current in metadata[1:]:
        if current.schema != first.schema or current.point_step != first.point_step:
            raise PcdError(
                "{}: PCD schema differs from {}".format(current.path, first.path))

    output_path = pcd_dir / _session_output_name(pcd_dir.parent.parent)
    if dry_run:
        total_bytes = sum(item.path.stat().st_size for item in metadata)
        print(
            "DRY RUN {}: would merge {} valid slices ({:.2f} MiB), "
            "preserve {} corrupt slices -> {}".format(
                pcd_dir, len(metadata), total_bytes / (1024.0 * 1024.0),
                corrupt_count, output_path.name))
        return "dry-run"

    total_points = sum(item.point_count for item in metadata)
    total_payload_bytes = sum(item.payload_bytes for item in metadata)
    required_bytes = total_payload_bytes + FREE_SPACE_RESERVE_BYTES
    free_bytes = shutil.disk_usage(str(pcd_dir)).free
    if free_bytes < required_bytes:
        raise PcdError(
            "{}: insufficient free space; need at least {:.2f} GiB, have {:.2f} GiB".format(
                pcd_dir, required_bytes / (1024.0 ** 3), free_bytes / (1024.0 ** 3)))

    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
                mode="wb", dir=str(pcd_dir), prefix=".{}-".format(output_path.name),
                suffix=".tmp", delete=False) as output_stream:
            temporary_path = Path(output_stream.name)
            output_stream.write(_merged_header(first, total_points))
            for item in metadata:
                with item.path.open("rb") as input_stream:
                    input_stream.seek(item.data_offset)
                    remaining = item.payload_bytes
                    while remaining:
                        chunk = input_stream.read(min(COPY_CHUNK_BYTES, remaining))
                        if not chunk:
                            raise PcdError("{}: unexpected end of PCD payload".format(item.path))
                        output_stream.write(chunk)
                        remaining -= len(chunk)
            output_stream.flush()
            os.fsync(output_stream.fileno())

        merged_metadata = inspect_binary_pcd(temporary_path)
        if merged_metadata.point_count != total_points or merged_metadata.payload_bytes != total_payload_bytes:
            raise PcdError("{}: merged PCD verification failed".format(temporary_path))

        os.replace(str(temporary_path), str(output_path))
        temporary_path = None
        for item in metadata:
            item.path.unlink()

        print(
            "MERGED {}: {} valid slices, {} points, preserved {} corrupt slices -> {}".format(
                pcd_dir, len(metadata), total_points, corrupt_count, output_path.name))
        return "merged"
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def find_pcd_directories(log_dir: Path) -> Sequence[Path]:
    if not log_dir.is_dir():
        raise PcdError("log directory does not exist: {}".format(log_dir))
    directories = []
    for session_dir in sorted(log_dir.iterdir()):
        if session_dir.is_symlink() or not session_dir.is_dir():
            continue
        pcd_dir = session_dir / "fastlivo2" / "pcd"
        if pcd_dir.is_dir():
            directories.append(pcd_dir)
    return directories


def find_active_fastlivo_session_roots(proc_root: Path = Path("/proc")) -> Sequence[Path]:
    session_roots = set()
    for process_dir in proc_root.iterdir():
        if not process_dir.name.isdigit():
            continue
        try:
            command_line = (process_dir / "cmdline").read_bytes()
            if b"fastlivo_mapping" not in command_line:
                continue
            environment = (process_dir / "environ").read_bytes().split(b"\0")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        prefix = b"FASTDRONE_SESSION_LOG_ROOT="
        for entry in environment:
            if entry.startswith(prefix):
                value = entry[len(prefix):].decode("utf-8", errors="surrogateescape")
                if value:
                    session_roots.add(Path(value).resolve())
                break
    return sorted(session_roots)


def parse_args(argv: Optional[Sequence[str]] = None):
    workspace = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description="Merge FAST-LIVO2 binary PCD slices per tmux session and delete verified inputs.")
    parser.add_argument(
        "--log-dir", type=Path, default=workspace / "log",
        help="Fast-Drone log root (default: %(default)s)")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="show what would be merged without creating or deleting files")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    counts = {"merged": 0, "skipped": 0, "empty": 0, "dry-run": 0, "failed": 0}
    try:
        directories = find_pcd_directories(args.log_dir.resolve())
    except PcdError as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 1

    active_pcd_directories = {
        (session_root / "fastlivo2" / "pcd").resolve()
        for session_root in find_active_fastlivo_session_roots()
    }

    for pcd_dir in directories:
        if not args.dry_run and pcd_dir.resolve() in active_pcd_directories:
            counts["failed"] += 1
            print(
                "ERROR {}: FAST-LIVO2 is still running for this session; "
                "stop mapping before merging".format(pcd_dir),
                file=sys.stderr)
            continue
        try:
            result = merge_pcd_directory(pcd_dir, dry_run=args.dry_run)
            counts[result] += 1
        except (OSError, ValueError, PcdError) as error:
            counts["failed"] += 1
            print("ERROR {}: {}".format(pcd_dir, error), file=sys.stderr)

    print(
        "Summary: merged={merged}, skipped={skipped}, empty={empty}, "
        "dry_run={dry-run}, failed={failed}".format(**counts))
    return 1 if counts["failed"] else 0


if __name__ == "__main__":
    sys.exit(main())
