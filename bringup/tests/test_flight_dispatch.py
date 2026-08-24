#!/usr/bin/env python3

import base64
import os
from pathlib import Path
import shlex
import signal
import subprocess
import tempfile
import time
import unittest
import uuid


WORKSPACE = Path(__file__).resolve().parents[2]
ENTRY = WORKSPACE / "bringup/flight.sh"
PROFILES = WORKSPACE / "bringup/profiles"
RUNTIME = WORKSPACE / "bringup/lib/tmux_runtime.sh"


def run_entry(*arguments, **environment):
    env = os.environ.copy()
    env.update(environment)
    return subprocess.run(
        ["bash", str(ENTRY), *arguments],
        check=False,
        text=True,
        capture_output=True,
        env=env,
    )


class FlightDispatchTest(unittest.TestCase):
    def test_dispatches_exactly_the_selected_profile(self):
        result = run_entry("pc_sim", FASTDRONE_DISPATCH_DRY_RUN="1")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            result.stdout,
            f"{PROFILES / 'pc_sim/tmux.sh'}\n",
        )

    def test_requires_one_profile_argument(self):
        missing = run_entry()
        extra = run_entry("pc_sim", "lidar")
        self.assertEqual(missing.returncode, 64)
        self.assertEqual(extra.returncode, 64)

    def test_rejects_unknown_profile(self):
        result = run_entry("unknown_vehicle")
        self.assertEqual(result.returncode, 66)
        self.assertIn("Unknown profile: unknown_vehicle", result.stderr)

    def test_lists_profiles_from_profile_directories(self):
        result = run_entry("--list-profiles")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            result.stdout.splitlines(),
            ["pc", "pc_sim", "xi35_10", "xi35_default", "xi35_euroc"],
        )


class ProfileTmuxRuntimeTest(unittest.TestCase):
    def test_ctrl_c_allows_graceful_exit_before_forced_cleanup(self):
        child_pid = None
        child_pgid = None
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            child_identity = temporary / "child-process"
            graceful_marker = temporary / "graceful-exit"
            child_body = (
                "trap '' HUP TERM; "
                f"printf '%s %s' \"$$\" \"$(ps -o pgid= -p $$)\" > "
                f"{shlex.quote(str(child_identity))}; "
                f"trap 'sleep 0.3; printf graceful > "
                f"{shlex.quote(str(graceful_marker))}; exit 0' INT; "
                "while :; do sleep 1; done"
            )
            pane_command = f"bash -c {shlex.quote(child_body)}"
            shell_body = (
                f"source {shlex.quote(str(RUNTIME))}; "
                f"run_managed_pane_command {shlex.quote(pane_command)}"
            )
            process = subprocess.Popen(
                ["bash", "-c", shell_body],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            try:
                for _ in range(100):
                    if child_identity.exists():
                        break
                    time.sleep(0.02)
                self.assertTrue(
                    child_identity.exists(),
                    "managed pane child did not start",
                )
                child_pid, child_pgid = map(
                    int, child_identity.read_text().split()
                )

                started = time.monotonic()
                process.send_signal(signal.SIGINT)
                process.wait(timeout=4)
                elapsed = time.monotonic() - started

                self.assertTrue(
                    graceful_marker.exists(),
                    "SIGTERM arrived before the SIGINT grace period ended",
                )
                self.assertLess(elapsed, 3.5)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
                if child_pgid and child_pgid != os.getpgrp():
                    try:
                        os.killpg(child_pgid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass

    def test_killing_tmux_session_stops_pane_process_group(self):
        socket_name = f"fastdrone_cleanup_{uuid.uuid4().hex}"
        tmux = ["tmux", "-L", socket_name]
        child_pid = None
        child_pgid = None

        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            workspace = temporary / "workspace"
            setup = workspace / "devel/setup.bash"
            setup.parent.mkdir(parents=True)
            setup.write_text("")
            gate = temporary / "layout-ready"
            gate.touch()
            process_identity = temporary / "child-process"
            fixture = temporary / "profile-tmux.sh"
            fixture.write_text(
                f'''#!/usr/bin/env bash
set -eu
WORKSPACE={shlex.quote(str(workspace))}
PROFILE_DIR={shlex.quote(str(temporary))}
PROFILE_NAME=cleanup_test
PROFILE_MODE=test
PROFILE_TMUX_SCRIPT={shlex.quote(str(fixture))}
ROS_MASTER_URI_DEFAULT=http://127.0.0.1:11311
ROS_IP_DEFAULT=127.0.0.1
source {shlex.quote(str(RUNTIME))}
run_tmux_profile "$@"
'''
            )
            child_body = (
                "trap '' HUP; "
                f"printf '%s %s' \"$$\" \"$(ps -o pgid= -p $$)\" > "
                f"{shlex.quote(str(process_identity))}; "
                "while :; do sleep 1; done"
            )
            pane_command = f"bash -c {shlex.quote(child_body)}"
            encoded_command = base64.b64encode(
                pane_command.encode()
            ).decode()

            subprocess.run(
                [
                    *tmux,
                    "-f",
                    "/dev/null",
                    "new-session",
                    "-d",
                    "-s",
                    "test",
                    (
                        f"FASTDRONE_TMUX_SOCKET={socket_name} "
                        f"bash {shlex.quote(str(fixture))} --pane-shell "
                        f"ready {shlex.quote(str(gate))} "
                        f"{encoded_command} auto"
                    ),
                ],
                check=True,
                text=True,
                capture_output=True,
            )
            try:
                for _ in range(100):
                    if process_identity.exists():
                        break
                    time.sleep(0.02)
                self.assertTrue(
                    process_identity.exists(),
                    "pane child did not start",
                )
                identity = process_identity.read_text().split()
                child_pid, child_pgid = map(int, identity)

                subprocess.run(
                    [*tmux, "kill-session", "-t", "test"],
                    check=True,
                    text=True,
                    capture_output=True,
                )
                for _ in range(100):
                    try:
                        os.kill(child_pid, 0)
                    except ProcessLookupError:
                        break
                    time.sleep(0.02)
                else:
                    self.fail(
                        f"pane child {child_pid} survived tmux exit"
                    )
            finally:
                subprocess.run(
                    [*tmux, "kill-server"],
                    check=False,
                    text=True,
                    capture_output=True,
                )
                if (
                    child_pgid
                    and child_pgid != os.getpgrp()
                ):
                    try:
                        os.killpg(child_pgid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass

    def test_attach_cleanup_disables_terminal_mouse_reporting(self):
        socket_name = f"fastdrone_attach_{uuid.uuid4().hex}"
        tmux = ["tmux", "-L", socket_name]
        subprocess.run(
            [
                *tmux,
                "-f",
                "/dev/null",
                "new-session",
                "-d",
                "-s",
                "test",
                "sleep 0.4",
            ],
            check=True,
            text=True,
            capture_output=True,
        )
        try:
            shell_body = (
                f"source {shlex.quote(str(RUNTIME))}; "
                f"FASTDRONE_TMUX_SOCKET={shlex.quote(socket_name)}; "
                "TERM=xterm; export TERM; SESSION=test; attach_tmux_session"
            )
            result = subprocess.run(
                [
                    "script",
                    "-qefc",
                    f"bash -c {shlex.quote(shell_body)}",
                    "/dev/null",
                ],
                check=False,
                capture_output=True,
                timeout=5,
            )

            self.assertEqual(result.returncode, 0, result.stderr.decode())
            self.assertIn(
                b"\x1b[?9l\x1b[?1000l\x1b[?1001l\x1b[?1002l"
                b"\x1b[?1003l\x1b[?1004l\x1b[?1005l\x1b[?1006l"
                b"\x1b[?1015l",
                result.stdout,
            )
        finally:
            subprocess.run(
                [*tmux, "kill-server"],
                check=False,
                text=True,
                capture_output=True,
            )

    def test_ros_setup_is_loaded_without_disabling_callers_nounset(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory)
            setup = workspace / "devel/setup.bash"
            setup.parent.mkdir()
            setup.write_text('SETUP_OBSERVED="$ROS_DISTRO"\n')

            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    '''
set -u
unset ROS_DISTRO
source "$1"
WORKSPACE="$2"
PROFILE_DIR="$2"
PROFILE_NAME=test
PROFILE_MODE=test
ROS_MASTER_URI_DEFAULT=http://127.0.0.1:11311
ROS_IP_DEFAULT=127.0.0.1
setup_profile_runtime_environment
case $- in
    *u*) printf 'nounset-on' ;;
    *) printf 'nounset-off' ;;
esac
printf '|%s' "${SETUP_OBSERVED-unset}"
''',
                    "runtime-test",
                    str(RUNTIME),
                    str(workspace),
                ],
                check=False,
                text=True,
                capture_output=True,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout, "nounset-on|")

    def test_runtime_uses_tmux_buffer_for_mouse_copy_and_paste(self):
        socket_name = f"fastdrone_test_{uuid.uuid4().hex}"
        tmux = ["tmux", "-L", socket_name]
        subprocess.run(
            [*tmux, "-f", "/dev/null", "new-session", "-d", "-s", "test"],
            check=True,
            text=True,
            capture_output=True,
        )
        try:
            configured = subprocess.run(
                [
                    "bash",
                    "-c",
                    f"source {RUNTIME!s}; "
                    f"FASTDRONE_TMUX_SOCKET={socket_name}; "
                    "SESSION=test; configure_tmux_session",
                ],
                check=False,
                text=True,
                capture_output=True,
            )
            self.assertEqual(configured.returncode, 0, configured.stderr)

            mouse = subprocess.run(
                [*tmux, "show-options", "-t", "test", "mouse"],
                check=True,
                text=True,
                capture_output=True,
            ).stdout
            clipboard = subprocess.run(
                [*tmux, "show-options", "-s", "set-clipboard"],
                check=True,
                text=True,
                capture_output=True,
            ).stdout
            copy_bindings = subprocess.run(
                [*tmux, "list-keys", "-T", "copy-mode-vi"],
                check=True,
                text=True,
                capture_output=True,
            ).stdout
            root_bindings = subprocess.run(
                [*tmux, "list-keys", "-T", "root"],
                check=True,
                text=True,
                capture_output=True,
            ).stdout

            self.assertEqual(mouse.strip(), "mouse on")
            self.assertEqual(clipboard.strip(), "set-clipboard off")
            drag_binding = next(
                line for line in copy_bindings.splitlines()
                if "MouseDragEnd1Pane" in line
            )
            self.assertIn("copy-selection-and-cancel", drag_binding)
            for mouse_button in ("MouseDown2Pane", "MouseDown3Pane"):
                paste_binding = next(
                    line for line in root_bindings.splitlines()
                    if mouse_button in line and "M-" + mouse_button not in line
                )
                self.assertIn("paste-buffer", paste_binding)
            exit_binding = next(
                line for line in root_bindings.splitlines()
                if "MouseDown1StatusRight" in line
            )
            self.assertIn("kill-session -t", exit_binding)
            self.assertIn("#{session_name}", exit_binding)
        finally:
            subprocess.run(
                [*tmux, "kill-server"],
                check=False,
                text=True,
                capture_output=True,
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
