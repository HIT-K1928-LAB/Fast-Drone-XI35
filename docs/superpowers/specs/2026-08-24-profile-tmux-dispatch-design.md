# Profile-Owned tmux Launch Design

## Goal

Replace the implicit `profile.env -> FLIGHT_PROFILE_SCRIPT -> build_user_layout`
chain with one public command and one readable tmux script per vehicle profile:

```bash
bash bringup/flight.sh <profile>
```

The public entry accepts exactly one profile argument. Localization and other
vehicle modes are selected only through that profile's `profile.env`.

## Structure

```text
bringup/
├── flight.sh
├── lib/tmux_runtime.sh
└── profiles/<profile>/
    ├── tmux.sh
    ├── profile.env
    ├── launch/
    └── config/
```

`flight.sh` validates the profile and executes its `tmux.sh`. Each profile
script loads its environment, defines its windows and panes, and invokes the
shared runtime. `tmux_runtime.sh` owns session creation, pane shells,
synchronization, manual/auto execution, status UI and mouse behavior; it has no
knowledge of ROS modules or localization algorithms.

## Profile Modes

- `pc_sim` reads `ODOMETRY_SOURCE=ground_truth|vins|lidar`.
- `pc`, `xi35_10`, and `xi35_default` read
  `LOCALIZATION_DEFAULT=vins_stereo|mocap|motion_capture`.
- `xi35_euroc` has the fixed mode `euroc`.

No public second argument may override these values. Session names remain
`fd_<profile>_<mode>`.

## Mouse and Clipboard

The runtime keeps tmux mouse support enabled for wheel history and pane
selection. Ptyxis/VTE ignores OSC52 writes, so the runtime disables tmux
clipboard synchronization. Mouse drag copies into the tmux buffer, and middle
or right click pastes that buffer. `Shift+drag` followed by the terminal's copy
shortcut is the separate native-terminal clipboard path.

No `xclip`, `xsel`, or `wl-copy` dependency is introduced.

## Compatibility and Errors

- `--help`, `--list-profiles`, and `completion` remain supported.
- Zero profile arguments, unknown profiles, extra public arguments and missing
  `tmux.sh` files fail before tmux is started.
- Existing sessions are attached rather than recreated.
- Existing profile launch/config files and original localization output topics
  remain unchanged.
- The old `bringup/tmux/flight.sh`, default layout and profile-specific
  `flight_profile.sh` files are removed after migration.

## Verification

Automated tests execute the real dispatcher with controlled profile fixtures,
render every real profile layout without starting ROS, reject public extra
arguments, and configure a real isolated tmux server to verify mouse,
clipboard and drag-copy settings. Existing launch/config tests and ROS launch
parsing must remain green.
