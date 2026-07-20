# Fast-Drone Bringup

`bringup` 用于统一管理不同无人机的整机配置，以及用 tmux 编排飞行相关进程。

```text
bringup/
├── profiles/                 # 每台无人机的配置档案
│   ├── xi35_default/
│   └── xi35_10/
└── tmux/
    └── flight.sh             # 飞行进程统一入口
```

基本使用：

```bash
# 查看帮助和当前可用机型
./bringup/tmux/flight.sh --help

# 使用默认机型和默认定位方式
./bringup/tmux/flight.sh

# 指定机型
./bringup/tmux/flight.sh xi35_10

# 完整指定机型、定位方式和视觉前端
./bringup/tmux/flight.sh xi35_10 vins_stereo xfeat
```

详细说明：

- [profiles 使用说明](profiles/README.md)
- [tmux 使用与维护说明](tmux/README.md)

配置原则：

- 机型相关配置放在 `bringup/profiles/<profile>/`，并提交到 Git。
- 功能包中的 `config/template/` 用作默认模板，不作为具体飞机的飞行配置。
- 启动时只选择一个 profile，VINS、PX4Ctrl 等模块应使用该 profile 下的配置。
- 不要在启动脚本中用 `sed -i` 修改受 Git 管理的配置文件。

