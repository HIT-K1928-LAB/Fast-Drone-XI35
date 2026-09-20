# 验证记录：2026-09-17，Orin / fd_runtime

## 已完成

- 在目标 aarch64 板端独立 CMake Release 编译成功，使用现有飞行工作区消息包。
- 原生 TensorRT 8.5.2.2 加载现有 `yopo_minco_fp32.engine`，不依赖 Python/Torch CUDA。
- `rospack find yopo_minco_planner`、launch 文件解析通过。
- OpenCV 实际使用 `/usr` 下的完整 **4.5.4**（core/imgproc/photo）；不是 `/usr/local/opencv-4.5.4` 下缺少 photo 的构建。
  板端 Python cv2 显示 4.2.0，不能用它的版本推断 C++ 链接版本；`ldd` 已确认链接 4.5。
- 数值对照：24 组两段求解、24 组候选解码、24 组观测编码；速度覆盖 0.3/0.5/1.0。
  使用板端 Python 源码提取的求解、CPU 解码与归一化函数作参考，测试不依赖 Torch 导入。
  首轮最大系数绝对差 `1.69e-8`，PVA 采样绝对差 `3.12e-6`，解码 `2.30e-6`，观测编码 `7.79e-6`，均通过脚本容差。
- 通道准入、最大 score 选择、空间限值拒绝和无效求解时间测试通过。
- TensorRT 与 ONNX Runtime CPU：3 组已归一化合成输入，全部三个输出通过 `rtol=1e-4, atol=1e-5`。
  最大输出绝对差约 `6.86e-6`；C++ 探针测得单次主机推理平均约 4.95–6.00 ms。
- 隔离 ROS master `127.0.0.1:11319` 行为测试通过：启动无指令、目标单独不执行、显式触发、仅调试话题、
  world/READY 消息、用户停止、传感器断流停止、数据恢复不自动恢复。
  合成测试使用专门放宽的轨迹限制，以覆盖输出/停止路径，未修改 `config/real.yaml`。
- 测试进程结束后，隔离 master 与测试节点已退出；未启动相机、PX4Ctrl、起飞或解锁。

## 测试范围限制

上述数值测试不是真实传感器或实飞验收，也未证明 PyTorch checkpoint 到 ONNX 导出的完整一致性。
TensorRT 对照使用合成输入，需补充真实深度/状态录包。
尚未验证默认轨迹限值的任务通过率、相机 FOV、相机平移、在线外参变化、KF 定位异常或实飞停车距离。
原有 Python 脚本和旧 YOPO 包均未修改。本包许可/维护者元数据仍待项目维护者确认后完善。

## 可复现命令

见 README 的独立构建与数值测试命令。ROS 隔离测试：

```bash
source /root/Fast-Drone-XI35/devel/setup.bash
P=/root/data/yopo-minco-flight-shallow
python3 "$P/src/planning/yopo_minco_planner/tests/verify_ros.py" \
  --node "$P/build_yopo_minco/devel/lib/yopo_minco_planner/yopo_minco_node" \
  --package "$P/src/planning/yopo_minco_planner" \
  --engine "$P/yopo_minco_sim/deployment/models/yopo_minco_fp32.engine" \
  --extrinsic /root/Fast-Drone-XI35/src/realflight_modules/VINS-Fusion-gpu/config/fast_drone_250.yaml
```

该测试拒绝 11311 端口及已有 master，不会接入已有飞行 ROS master。
