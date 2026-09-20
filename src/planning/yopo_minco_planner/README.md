# YOPO–MINCO 原生 TensorRT / ROS1 部署包

独立包，不修改旧 `yopo_planner`、Python 入口、PX4Ctrl 或已有定位配置。
包的目录布局参考 HIT-K1928-LAB/Fast-Drone-XI35 `board` 分支 `5cd90b8`；
数值契约以板端 `third_party/YOPO/YOPO` 的 MINCO 实现为准。
这不是重新训练的模型，也不是旧版 Poly5 更换权重。

## 文件职责

- `src/yopo_node.cpp`：ROS 输入、深度预处理、外参、任务门控、10 Hz 规划和 50 Hz 指令。
- `src/yopo_engine.cpp`：TensorRT 8 原生 engine 反序列化、完整尺寸/类型校验、显存与推理。
- `src/yopo_planner.cpp`：归一化、图像候选顺序、14 通道解码、预测通道筛选和轨迹检查。
- `src/minco_solver.cpp`：两段五次多项式，起终点 PVA、中间点及 C4 连续约束，位置到 snap 采样。
- `tests/`：Python 参考数值对照、纯推理验证。探针不创建 ROS 节点或发布飞行指令。

## 模型契约

| 张量 |  绑定尺寸 |
|---|---|
| depth | 1×1×96×160 |
| obs | 1×9×3×5 |
| endstate | 1×14×3×5 |
| score | 1×3×5 |
| radius | 1×20×3×5 |

engine 仅执行神经网络，解码、筛选、MINCO 均在 C++ 中执行。
要求静态、线性布局、显式 batch；尺寸不符即报错，不自动重建或覆盖用户模型。
输入深度仍按当前 MINCO 的 20 m 归一化，不照搬旧 YOPO 的 5 m。
输出已经经过网络激活，解码不重复 tanh。
候选图像索引映射为 `14 - image_index`；topk=1，在通过 `min(mu-b)` 筛选的候选中取最大 score。
无合格候选或所选轨迹越界时停止，不静默改为另一种选轨策略。

`velocity` 同时影响速度/加速度归一化及两段时间，不是最终轨迹的硬速度上限。
`max_speed/max_acceleration/max_jerk` 是额外检查阈值。
实际世界坐标的网络观测与 MINCO 起点使用同一快照；默认实测位置/速度，参考加速度，首帧加速度为零。
开启 `plan_from_reference` 后，位置/速度/加速度均取当前参考轨迹。

## 构建（Orin 的 fd_runtime 容器内）

```bash
cd /root/data/yopo-minco-flight-shallow/src/planning/yopo_minco_planner
bash scripts/build_standalone.sh
source /root/data/yopo-minco-flight-shallow/build_yopo_minco/devel/setup.bash
```

仅构建这个包，依赖现有 `/root/Fast-Drone-XI35/devel` 的消息和 ROS 环境。
不运行整个新仓库的 catkin 编译，不替换已验证的飞行二进制。
OpenCV 使用系统完整版本，避免依赖板端缺少 `photo` 模块的自定义版本。
可以通过 `FLIGHT_SETUP`、`YOPO_MINCO_BUILD_DIR`、`YOPO_MINCO_OPENCV_DIR` 覆盖路径。

## 默认旁路启动

先按现有流程提供相机、KF 里程计。本 launch 不启动相机、MAVROS、PX4Ctrl，也不解锁、起飞。

```bash
source /root/Fast-Drone-XI35/devel/setup.bash
source /root/data/yopo-minco-flight-shallow/build_yopo_minco/devel/setup.bash
roslaunch yopo_minco_planner yopo_minco.launch
```

默认仅发布 `/yopo_minco/debug/position_cmd`；禁止通过 remap 把调试输出改到控制话题。
目标由 `/move_base_simple/goal` 接收，必须是 `world` 坐标，保留明确的 z 值。
设置目标后还必须给 `/traj_start_trigger` 一次 PoseStamped 触发；触发消息的坐标不作为目标。
停止使用 `/yopo_minco/stop` 的 Empty 消息。停止后不自动恢复，需要重新触发。
调试模式不要求 FCU 解锁，但仍要求新鲜且有效的里程计/深度和规定的高度/速度范围。
可在台架上提供满足条件的真实里程计，或在隔离 ROS master 下重放正确时间戳的录包。

默认数据接口：

- `/kf_fusion/kf_imu_odom`：world 位姿和世界系线速度。
- `/camera/depth/image_rect_raw`：640×480，16UC1 毫米或 32FC1 米。
- `/mavros/state`：仅真实执行需要 connected、armed、OFFBOARD 且数据新鲜。
- `/yopo_minco/status`：IDLE/GOAL_SET/ACTIVE/STOP 原因。
- `/yopo_minco/best_trajectory`：可选 PoseArray 可视化。

`execute:=true` 才创建 `/position_cmd` 发布者，且启动时和运行中检查其他发布者。
这只是软件开关，不代表完成实飞验证。代码不直接读取 PX4Ctrl FSM，仍需核对实际悬停/指令接管状态。
无效输入、计划过期、超限和停止均采用释放指令，依赖 PX4Ctrl 超时回悬停；不是主动刹车。

## 数值测试

```bash
PKG=/root/data/yopo-minco-flight-shallow/src/planning/yopo_minco_planner
BIN=/root/data/yopo-minco-flight-shallow/build_yopo_minco/devel/lib/yopo_minco_planner
MODEL=/root/data/yopo-minco-flight-shallow/yopo_minco_sim/deployment/models
python3 "$PKG/tests/verify_numeric.py" --probe "$BIN/yopo_minco_numeric_probe" \
  --yopo-root /root/data/yopo-minco-flight-shallow/third_party/YOPO/YOPO
python3 "$PKG/tests/verify_engine.py" --probe "$BIN/yopo_minco_engine_probe" \
  --engine "$MODEL/yopo_minco_fp16.engine" --onnx "$MODEL/yopo_minco_epoch50.onnx"
```

## 尚需实机数据验证的边界

- 外参读取 `body_T_cam0` 的旋转并组合 optical→YOPO 轴转换；目前不应用平移。
  必须确认 cam0 对应深度相机及 KF 机体系。只读取配置，不跟随 VINS 在线外参优化。
- 深度直接缩放到网络尺寸，不自动校正相机 FOV；真实内参与训练 FOV 的匹配仍需确认。
- 相机空洞/无效比例检查不等于障碍检测完整性；预测 corridor 不是几何距离保证。
- 轨迹限制是 20 ms 网格采样检查，不是连续时间极值或碰撞证明。
- 初始速度、高度、测试范围均是待验证的测试参数，可能拒绝大部分网络轨迹；不要通过盲目放宽限制绕过拒绝。
- 本包只作短距离单目标、无自动恢复部署；不包含完整地图碰撞检查或主动制动轨迹。
- 当前使用 ROS1 Noetic、TensorRT 8 的接口；TensorRT 10 需要单独移植。
- 发布前应核对原始算法/仓库许可并补全 package.xml 的许可及维护信息。
