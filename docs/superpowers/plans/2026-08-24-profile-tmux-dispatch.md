# Profile-Owned tmux Dispatch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provide `bash bringup/flight.sh <profile>` as the only public flight entry, with one tmux layout script per profile and reliable tmux mouse copy/paste.

**Architecture:** The public dispatcher only resolves a profile and executes its `tmux.sh`. Profile scripts own environment and layout; a source-only runtime owns tmux mechanics and reloads the same profile script for pane shells.

**Tech Stack:** Bash, tmux 3.0a, ROS Noetic launch files, Python `unittest`.

**Spec:** `docs/superpowers/specs/2026-08-24-profile-tmux-dispatch-design.md`

## Global Constraints

- The public entry accepts exactly one profile argument; mode changes come only from `profile.env`.
- Preserve each localization algorithm's existing output topic.
- Keep mouse wheel history and dependency-free tmux-buffer copy/paste.
- Preserve all pre-existing user changes in the dirty worktree.
- Do not create or attach a real production tmux session during automated tests.

---

### Task 1: Public Dispatcher Contract

**Files:**
- Create: `bringup/flight.sh`
- Create: `bringup/tests/test_flight_dispatch.py`

**Interfaces:**
- Consumes: `bringup/profiles/<profile>/tmux.sh`
- Produces: `bringup/flight.sh <profile>` and the existing help/list/completion commands.

- [ ] **Step 1: Write the failing dispatcher tests**

```python
def test_dispatcher_executes_only_selected_profile(self):
    result = run_dispatcher("pc_sim", env={"FASTDRONE_DISPATCH_DRY_RUN": "1"})
    self.assertEqual(result.stdout.strip(), str(PROFILES / "pc_sim/tmux.sh"))

def test_dispatcher_rejects_mode_override(self):
    result = run_dispatcher("pc_sim", "lidar", check=False)
    self.assertEqual(result.returncode, 64)
```

- [ ] **Step 2: Run tests and confirm failure because `bringup/flight.sh` does not exist**

Run: `python3 -m unittest bringup.tests.test_flight_dispatch -v`

- [ ] **Step 3: Implement the dispatcher**

Implement profile enumeration, exactly-one-argument validation, unknown/missing
profile errors, dry-run dispatch for tests, and `exec "$PROFILE_TMUX"`.

- [ ] **Step 4: Run the dispatcher tests**

Run: `python3 -m unittest bringup.tests.test_flight_dispatch -v`

---

### Task 2: Shared Runtime and Clipboard Behavior

**Files:**
- Create: `bringup/lib/tmux_runtime.sh`
- Extend: `bringup/tests/test_flight_dispatch.py`

**Interfaces:**
- Consumes: profile globals `PROFILE_NAME`, `PROFILE_MODE`, `PROFILE_DIR`, `PROFILE_TMUX_SCRIPT`, `ROS_MASTER_URI_DEFAULT`, and `ROS_IP_DEFAULT`; profile functions `build_profile_layout()` and optional `setup_profile_environment()`.
- Produces: `new_window`, `add_pane`, `configure_tmux_session`, and `run_tmux_profile`.

- [ ] **Step 1: Write a failing isolated-server mouse test**

```python
def test_runtime_enables_mouse_and_osc52_clipboard(self):
    options = configure_isolated_tmux_server()
    self.assertEqual(options["mouse"], "on")
    self.assertEqual(options["set-clipboard"], "on")
    self.assertIn("copy-selection-and-cancel", options["drag_binding"])
    self.assertEqual(options["set_clipboard"], "off")
```

- [ ] **Step 2: Run the test and confirm failure because the runtime is missing**

Run: `python3 -m unittest bringup.tests.test_flight_dispatch.ProfileTmuxRuntimeTest -v`

- [ ] **Step 3: Move generic tmux mechanics into the runtime**

Implement a `tmux_cmd()` wrapper honoring `FASTDRONE_TMUX_SOCKET`, pane command
encoding, profile-aware pane reload, layout gating, status window, attach logic,
mouse options, drag-copy bindings and middle/right-click tmux-buffer paste.

- [ ] **Step 4: Verify runtime tests pass on an isolated tmux server**

Run: `python3 -m unittest bringup.tests.test_flight_dispatch.ProfileTmuxRuntimeTest -v`

---

### Task 3: Per-Profile tmux Scripts

**Files:**
- Create: `bringup/profiles/{pc,xi35_10,xi35_default,pc_sim,xi35_euroc}/tmux.sh`
- Modify: each matching `profile.env`
- Modify: `bringup/tests/test_profile_launch_layouts.py`
- Modify: `bringup/tests/test_indoor1_fastlivo2.py`

**Interfaces:**
- Consumes: `bringup/lib/tmux_runtime.sh` and each profile's `launch/` directory.
- Produces: one directly executable layout script per profile and a `PROFILE_MODE` selected only from profile configuration.

- [ ] **Step 1: Change layout tests to execute each profile `tmux.sh --print-layout`**

```python
for profile, mode in {"pc_sim": "lidar", "xi35_default": "vins_stereo"}.items():
    result = subprocess.run([PROFILES/profile/"tmux.sh", "--print-layout"], ...)
    self.assertIn(f"MODE|{mode}", result.stdout)
    self.assertNotIn(":=", result.stdout)
```

- [ ] **Step 2: Run tests and confirm all profiles fail because `tmux.sh` is missing**

Run: `python3 -m unittest bringup.tests.test_profile_launch_layouts bringup.tests.test_indoor1_fastlivo2 -v`

- [ ] **Step 3: Implement profile scripts**

Hardware scripts use `LOCALIZATION_DEFAULT`; `pc_sim` uses `ODOMETRY_SOURCE`;
EuRoC sets `PROFILE_MODE=euroc`. Each script defines only environment hooks and
its explicit `new_window`/`add_pane` layout, then calls `run_tmux_profile "$@"`.

- [ ] **Step 4: Remove `FLIGHT_PROFILE_SCRIPT` from every `profile.env`**

Keep ROS network, camera/GPU values and mode selection only.

- [ ] **Step 5: Run all layout and simulation tests**

Run: `python3 -m unittest discover -s bringup/tests -p 'test_*.py' -v`

---

### Task 4: Remove Old Entry and Update Documentation

**Files:**
- Delete: `bringup/tmux/flight.sh`
- Delete: `bringup/tmux/default_flight_profile.sh`
- Delete: `bringup/profiles/pc_sim/flight_profile.sh`
- Delete: `bringup/profiles/xi35_euroc/flight_profile.sh`
- Move/update: `bringup/tmux/README.md` into `bringup/README.md`
- Modify: `bringup/profiles/README.md`, `docs/simulation.md`

**Interfaces:**
- Consumes: completed dispatcher/runtime/profile scripts.
- Produces: documentation with only `bash bringup/flight.sh <profile>` public examples.

- [ ] **Step 1: Remove old scripts and stale references**

Run after editing: `rg -n 'bringup/tmux/flight.sh|FLIGHT_PROFILE_SCRIPT|flight_profile.sh' bringup docs`
Expected: no runtime or documentation matches.

- [ ] **Step 2: Document profile-only mode changes and clipboard behavior**

Document `ODOMETRY_SOURCE`/`LOCALIZATION_DEFAULT`, tmux-buffer mouse
copy/paste, and the separate `Shift+drag` system-clipboard path.

- [ ] **Step 3: Run complete verification**

```bash
python3 -m unittest discover -s bringup/tests -p 'test_*.py' -v
bash -n bringup/flight.sh bringup/lib/tmux_runtime.sh bringup/profiles/*/tmux.sh
git diff --check
```

Parse all profile ROS launch files in `fd-runtime-pc-sim` with
`roslaunch --nodes`; the pre-existing unavailable EuRoC Foxglove dependency is
the only allowed skip.
