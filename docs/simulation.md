# Iris 仿真模型与传感器配置

本文记录 Fast-Drone-XI35 当前 Gazebo Classic 仿真中可用于 Iris 四旋翼的 SDF 模型、传感器参数和 ROS 话题。参数以仓库中的 SDF 文件为准；修改模型后，应同步更新本文和对应定位算法配置。

## 启动入口与模型选择

统一启动入口为：

```bash
bash bringup/flight.sh pc_sim
```

`simulator` 窗口固定启动 PX4/XTDrone 原生场景：

```bash
roslaunch px4 indoor1.launch
```

该 launch 当前生成 `iris_3d_gpu_lidar`。`bringup/profiles/pc_sim/profile.env` 中的 `ODOMETRY_SOURCE` 只决定启用哪套定位方案：

| `ODOMETRY_SOURCE` | Gazebo SDF | 定位输入 | PX4Ctrl 里程计 |
|---|---|---|---|
| `ground_truth` | `iris_3d_gpu_lidar` | Gazebo 模型真值 | `/gazebo/iris_0/odometry` |
| `vins` | `iris_3d_gpu_lidar` | 双目图像和 IMU | `/vins_fusion/imu_propagate` |
| `lidar` | `iris_3d_gpu_lidar` | 双目左图、32 线 GPU 激光和 IMU | `/LIVO2/imu_propagate` |

选择 `lidar` 时，`simulator` 的并列 pane 还会启动 `velodyne_to_fastlivo.launch`，给 Gazebo 点云补充 FAST-LIVO2 所需的逐点 `time` 字段。

在 launch 文件中，`vehicle` 和 `sdf` 的作用不同：

```xml
<arg name="vehicle" value="iris"/>
<arg name="sdf" value="iris_3d_gpu_lidar"/>
```

- `vehicle="iris"` 选择 PX4 的 Iris 四旋翼机架和混控配置。
- `sdf="iris_3d_gpu_lidar"` 选择 Gazebo 中生成的机身和传感器组合。
- 只更换 Iris 的传感器时，保留 `vehicle="iris"`，只修改 `sdf`。
- 更换为固定翼、VTOL 或其他机架时，需要同时修改 `vehicle`、`sdf`、ROS 命名空间和控制脚本；当前 PX4Ctrl 配置面向多旋翼。

`pc_sim.launch` 支持直接指定模型。例如启动双目模型：

```bash
roslaunch simulation_bridges pc_sim.launch sdf:=iris_stereo_camera
```

项目派生模型位于 `simulation_bridges/models`，启动时还要指定模型根目录：

```bash
roslaunch simulation_bridges pc_sim.launch \
  sdf:=iris_realsense_gpu_lidar \
  model_root:="$(rospack find simulation_bridges)/models"
```

## 项目派生模型

以下模型是本项目保留的 RealSense 和 Livox 派生方案；当前 `pc_sim` 的 `lidar` 模式改为直接使用 PX4/XTDrone 的 `iris_3d_gpu_lidar`。

| SDF 模型 | 传感器组合 | 主要用途 |
|---|---|---|
| `iris_realsense_gpu_lidar` | RealSense、32 线 GPU 激光、GPS、机体 IMU、RealSense IMU、碰撞传感器 | RealSense 与 GPU 激光组合的备用模型 |
| `iris_realsense_mid360` | RealSense、Mid-360 扫描轨迹激光、GPS、机体 IMU、RealSense IMU、碰撞传感器 | 验证官方 Livox Gazebo 插件和 Mid-360 数据链 |

两种模型使用相同的安装位姿：

| 设备 | 相对 `base_link` 的位姿 `x y z roll pitch yaw` |
|---|---|
| RealSense | `0.10 0 0 0 0 0` |
| GPU 激光或 Mid-360 | `0.05 0 0.095 0 0 0` |
| RealSense 模拟 IMU | RealSense 模型内部 `0 0 0.30 0 0 0` |

### 32 线 GPU 激光

模型组件为 `3d_gpu_lidar_fastlivo`，使用 Gazebo `gpu_ray` 和 `libgazebo_ros_velodyne_gpu_laser.so`。它是保留在 `simulation_bridges` 中的派生备用版本。

| 配置项 | 当前值 |
|---|---|
| 更新率 | `10 Hz` |
| 水平扫描 | `512` 点，`-180°` 至 `180°` |
| 垂直扫描 | `32` 线，`-15°` 至 `15°` |
| 测距范围 | `0.2–30 m` |
| 距离分辨率 | `0.01 m` |
| 高斯噪声 | `0.008 m` |
| Gazebo 类型 | `gpu_ray`，使用 GPU 渲染激光 |
| ROS 消息 | `sensor_msgs/PointCloud2` |
| 原始 ROS 话题 | `/iris_0/velodyne_points` |
| 转换后 ROS 话题 | `/iris_0/velodyne_points_fastlivo` |
| 坐标系 | `laser_3d` |

Gazebo 原始点云包含 `x/y/z/intensity/ring`，但没有逐点 `time`。转换节点保留原字段并追加 `float32 time`，单位为微秒，按 `0.1 s` 扫描周期生成 `0–100000 μs` 的偏移时间。FAST-LIVO2 使用 `lidar_type: 2`、`scan_line: 32` 并订阅转换后的话题。对应配置为 `bringup/profiles/pc_sim/config/fastlivo2/gpu_lidar_sim.yaml`。

转换节点的输入话题、输出话题和扫描周期由 `bringup/profiles/pc_sim/launch/lidar_bridge.launch` 配置。tmux 只启动该 profile launch，不再追加内联参数。

### Mid-360 扫描轨迹激光

模型组件为 `livox_mid360_sim`，使用 `liblivox_laser_simulation.so` 和 `scan_mode/mid360.csv` 扫描轨迹，不是简单的规则 32 线雷达。

| 配置项 | 当前值 |
|---|---|
| 更新率 | `10 Hz` |
| 水平角范围 | `0–360°` |
| 垂直角范围 | 约 `-7.22°` 至 `55.22°` |
| 插件每帧采样数 | `24000` |
| 测距范围 | `0.1–200 m` |
| 距离分辨率 | `0.002 m` |
| 高斯噪声标准差 | `0.01 m` |
| 原始话题 | `/scan`，`sensor_msgs/PointCloud` |
| 转换后话题 | `/livox/lidar`，`livox_ros_driver2/CustomMsg` |

FAST-LIVO2 不能直接消费 Gazebo 插件的 `sensor_msgs/PointCloud`。使用 Mid-360 模型时，需要同时启动：

```bash
roslaunch simulation_bridges livox_to_fastlivo.launch
```

该转换节点按 `0.1 s` 扫描周期生成 Livox `CustomMsg` 中的 `offset_time`。对应算法配置为 `bringup/profiles/pc_sim/config/fastlivo2/mid360_sim.yaml`。

### RealSense 组合传感器

`iris_realsense_gpu_lidar` 和 `iris_realsense_mid360` 都引用同一个 `realsense_camera` 模型。

| 传感器 | 更新率 | 分辨率/视场角 | 主要话题 |
|---|---:|---|---|
| 左右目彩色相机 | `30 Hz` | `752×480`、RGB、水平视场角 `90°`、基线 `0.12 m` | `/iris_0/stereo_camera/left/image_raw`、`/iris_0/stereo_camera/right/image_raw` |
| 彩色/深度相机 | `20 Hz` | `640×480`、水平视场角 `60°`、有效点云 `0.05–10 m` | `/iris_0/realsense/depth_camera/color/image_raw`、`/iris_0/realsense/depth_camera/depth/image_raw`、`/iris_0/realsense/depth_camera/depth/points` |
| 模拟 IMU | `500 Hz` | `sensor_msgs/Imu` | `/iris_0/imu_gazebo` |

RealSense 组合目前不是 `pc_sim` 的默认 FAST-LIVO2 模型；切换回该模型时，需要同时切换主配置中的图像话题、相机内参和传感器外参。

## PX4/XTDrone 自带 Iris 模型

下表中的模型位于 `third_party/px4_sitl/Tools/sitl_gazebo/models`。表中的“基础 Iris”已经包含 PX4 所需的机体 IMU、气压计、磁力计、电机和 MAVLink 插件；这里只列出额外或有区别的传感器。

| SDF 模型 | 额外传感器或用途 |
|---|---|
| `iris` | GPS；最基础的 Iris 模型 |
| `iris_2d_lidar` | Hokuyo 2D 激光、单点测距仪、ROS IMU、GPS |
| `iris_3d_gpu_lidar` | GPU 32 线 3D 激光、双目 RGB、ROS IMU、GPS |
| `iris_3d_lidar` | CPU 32 线 3D 激光、双目 RGB、ROS IMU、GPS |
| `iris_depth_camera` | 前向深度相机 |
| `iris_downward_depth_camera` | 下视深度相机 |
| `iris_downward_camera` | 下视单目相机、ROS IMU、GPS |
| `iris_stereo_camera` | 双目 RGB、深度图、深度点云、ROS IMU、GPS、碰撞传感器 |
| `iris_realsense_camera` | RealSense 双目、彩色/深度相机、RealSense IMU、GPS |
| `iris_realsense_livox` | RealSense、Livox Avia、GPS、碰撞传感器；当前 SDF 仍引用 `livox_avia` |
| `iris_triple_depth_camera` | 前、左前、右前三个低分辨率深度相机和下向测距仪 |
| `iris_zhihang` | RealSense、下视单目、单点测距仪、ROS IMU、GPS |
| `iris_fpv_cam` | FPV RGB 相机 |
| `iris_opt_flow` | PX4Flow 光流相机和下向测距仪 |
| `iris_opt_flow_mockup` | 光流测试机架和下向测距仪，不包含完整 PX4Flow 相机模型 |
| `iris_rplidar` | RPLIDAR 2D 激光、光流相机和下向测距仪 |
| `iris_foggy_lidar` | 带强噪声/雾天效果的 2D 激光 |
| `iris_obs_avoid` | 下向测距仪和碰撞传感器，用于避障接口测试 |
| `iris_irlock` | IRLock 逻辑相机和下向测距仪，用于精确降落 |
| `iris_dual_gps` | 双 GPS |
| `iris_vision` | 视觉定位专用 PX4 参数模型，不额外挂载相机 |
| `iris_ctrlalloc` | Control Allocation 测试模型，不额外挂载传感器 |
| `iris_rtps` | RTPS 通信测试模型，不额外挂载传感器 |
| `iris_hitl` | HITL 专用模型，不用于常规 `pc_sim` |

### indoor1 双目 RGB 相机

`iris_stereo_camera`、`iris_3d_gpu_lidar` 和 `iris_3d_lidar` 通过 `model://stereo_camera` 引用 `stereo_camera/model.sdf`。

| 配置项 | 当前值 |
|---|---|
| 更新率 | `30 Hz` |
| 左右目分辨率 | `640×480` |
| 编码 | `rgb8` |
| 水平视场角 | `90°` |
| 双目基线 | `0.12 m` |
| 裁剪范围 | `0.1–300 m` |
| 相机内参 | `fx=320`、`fy=320`、`cx=320`、`cy=240` |
| 畸变 | `k1=-0.1`、`k2=0.01`、`p1=0.00005`、`p2=-0.0001` |
| 坐标系 | `stereo_camera_frame` |

主要话题为：

```text
/iris_0/stereo_camera/left/image_raw
/iris_0/stereo_camera/left/camera_info
/iris_0/stereo_camera/right/image_raw
/iris_0/stereo_camera/right/camera_info
```

当前 `iris_3d_gpu_lidar` 不发布深度图。FAST-LIVO2 使用左目图像、转换后的 GPU 点云和 `/iris_0/imu_gazebo`；相机参数来自 `bringup/profiles/pc_sim/config/fastlivo2/camera_stereo_sim.yaml`。

### 3D 激光

PX4/XTDrone 自带的 GPU 和 CPU 3D 激光扫描参数相同，区别是 Gazebo 传感器与插件实现：

| 配置项 | `3d_gpu_lidar` | `3d_lidar` |
|---|---|---|
| Gazebo 类型 | `gpu_ray` | `ray` |
| 插件 | `libgazebo_ros_velodyne_gpu_laser.so` | `libgazebo_ros_velodyne_laser.so` |
| 更新率 | `10 Hz` | `15 Hz` |
| 水平扫描 | `512` 点，`360°` | `512` 点，`360°` |
| 垂直扫描 | `32` 线，`-15°` 至 `15°` | `32` 线，`-15°` 至 `15°` |
| 测距范围 | `0.2–30 m` | `0.2–30 m` |
| 距离分辨率 | `0.01 m` | `0.01 m` |
| 点云话题 | `/velodyne_points` | `/velodyne_points` |
| 坐标系 | `laser_3d` | `laser_3d` |

GPU 版本通常明显快于 CPU `ray` 版本，但需要容器能够访问显卡和 OpenGL。当前 PX4/XTDrone 原始 GPU 组件已经配置为 FAST-LIVO2 所需的 `10 Hz`。

### 2D 激光与单点测距仪

| 组件 | 更新率 | 扫描/量程 | 噪声 | 话题与坐标系 |
|---|---:|---|---|---|
| `hokuyo_lidar` | `500 Hz` | `512` 点、`360°`、`0.5–20 m`、分辨率 `0.01 m` | 插件高斯噪声 `0.01` | `scan`，`laser_2d` |
| `rplidar` | `10 Hz` | `360` 点、`360°`、`0.2–6 m`、分辨率 `0.05 m` | 标准差 `0.01 m` | `laser/scan`，`rplidar_link` |
| `foggy_lidar` | `5.5 Hz` | `360` 点、约 `360°`、`0.06–120 m`、分辨率 `0.05 m` | 标准差 `1.0 m` | 雾天/强噪声测试 |
| `laser_rangefinder` | `500 Hz` | 单点、`0.01–20 m`、分辨率 `0.01 m` | 无显式噪声 | `distance`，`laser_1d` |
| `lidar` | `20 Hz` | 单点、`0.06–35 m`、分辨率 `0.01 m` | 标准差 `0.02 m` | PX4 距离传感器插件 |

这里的 `lidar` 是单点下向距离传感器，不是 2D 或 3D 点云雷达。

### 其他相机和感知传感器

| 组件 | 关键配置 | 用途 |
|---|---|---|
| `depth_camera` | `640×480` RGB/深度，传感器 `30 Hz`、ROS 插件 `20 Hz`，水平视场角约 `59°`，点云有效范围 `0.2–20 m` | 前向或下视深度感知 |
| `monocular_camera` | `1280×720`、`30 Hz`、水平视场角 `120°`、图像噪声标准差 `0.001` | 下视单目、视觉算法测试 |
| `fpv_cam` | `320×240`、`30 Hz`、水平视场角约 `60°` | FPV 图像和 GStreamer 视频 |
| `px4flow` | `64×64` 灰度、`100 Hz`、水平视场角约 `14.3°` | PX4 光流插件 |
| `flow_cam` | `64×64` 灰度、`100 Hz`、水平视场角约 `5.0°` | RPLIDAR 模型中的光流输入 |
| `irlock` | `50 Hz`、水平视场角约 `35°`、话题 `/irlock` | 红外信标精确降落 |
| `bumper_sensor` | 接触传感器 `30 Hz` | 碰撞检测 |

`iris_triple_depth_camera` 的三路深度相机均为 `64×48`、`20 Hz`、水平视场角约 `59°`、裁剪范围 `0.5–18 m`，分别发布在 `camera_front`、`camera_left` 和 `camera_right` 命名空间。

### GPS 与 ROS IMU

| 组件 | 更新率 | 说明 |
|---|---:|---|
| `gps` | `5 Hz` | 水平位置噪声标准差 `1 m`、垂直位置 `1 m`、水平速度 `0.1 m/s`、垂直速度 `0.2 m/s` |
| `imu_gazebo` | `500 Hz` | 发布 `imu_gazebo`，坐标系 `imu_link_stereo`；VINS 和 FAST-LIVO2 仿真均使用 `/iris_0/imu_gazebo` |

## 选择建议

| 测试目标 | 推荐 SDF | 原因 |
|---|---|---|
| 只验证轨迹规划和控制 | `iris_stereo_camera` 加 Gazebo 真值 | 排除定位算法误差，最容易定位规划或控制问题 |
| 验证 VINS 和轨迹规划联调 | `iris_stereo_camera` | 图像、IMU、分辨率和 VINS profile 已配套 |
| 验证 FAST-LIVO2 激光、图像和 IMU 融合 | `iris_3d_gpu_lidar` | 10 Hz/32 线 GPU 点云、30 Hz 左目图像和 500 Hz IMU 已配套 |
| 验证 Mid-360 数据格式和扫描轨迹 | `iris_realsense_mid360` | 使用 Mid-360 CSV 扫描模式和 Livox `CustomMsg` 转换链 |
| 测试 2D 建图或平面避障 | `iris_2d_lidar` 或 `iris_rplidar` | 直接提供 `LaserScan` |
| 测试深度相机避障 | `iris_depth_camera` 或 `iris_triple_depth_camera` | 直接提供深度图和点云 |

开发轨迹规划代码时，建议先用真值确认规划器和 PX4Ctrl，再依次切换到 VINS 或 FAST-LIVO2。这样可以把规划问题、控制问题和定位漂移分开排查。

## 运行后检查

列出当前 Iris 相关传感器话题：

```bash
rostopic list | grep -E 'iris_0|velodyne|livox|scan'
```

检查频率：

```bash
rostopic hz /iris_0/stereo_camera/left/image_raw
rostopic hz /iris_0/imu_gazebo
rostopic hz /iris_0/velodyne_points
rostopic hz /iris_0/velodyne_points_fastlivo
```

检查图像和点云字段：

```bash
rostopic echo -n 1 /iris_0/stereo_camera/left/image_raw
rostopic echo -n 1 /iris_0/stereo_camera/depth/image_raw
rostopic echo -n 1 /iris_0/velodyne_points
rostopic echo -n 1 /iris_0/velodyne_points_fastlivo
```

## 配置源文件

- `bringup/profiles/pc_sim/profile.env`：运行环境和定位方式选择。
- `bringup/profiles/pc_sim/tmux.sh`：tmux 窗口、定位分支和启动顺序。
- `bringup/profiles/pc_sim/launch/`：indoor1、点云转换、定位、控制与规划的 ROS 连接配置。
- `bringup/profiles/pc_sim/config/vins/`：VINS 图像、IMU、内参和外参配置。
- `bringup/profiles/pc_sim/config/fastlivo2/`：FAST-LIVO2 仿真内参、外参和算法配置。
- `src/simulation/uav_simulator/simulation_bridges/models/`：本项目派生的 GPU 激光和 Mid-360 模型。
- `src/simulation/uav_simulator/simulation_bridges/launch/velodyne_to_fastlivo.launch`：GPU Velodyne 点云 `time` 字段转换入口。
- `third_party/px4_sitl/Tools/sitl_gazebo/models/`：PX4/XTDrone 自带模型和传感器组件。


# 所有仿真环境总览

仿真环境可以通过修改/third_party/px4_sitl/launch/*.launch中的world变量来修改，也可以自己新建launch文件。

## indoor1
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785660967172-714e2f8c-9354-41e8-a3e8-89a8f68c4372.png)

## indoor2
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785660995090-d1994aab-ec80-44c3-b808-37ad1bbaed50.png)

## indoor3
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785661029249-45a7e1c8-07a9-45d2-9063-ed972883b6aa.png)

## indoor4
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785661065818-5db985d9-8d98-4320-9e94-e7c0e1d6ef7d.png)

## indoor5
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785661098276-e53f09e3-9730-40ce-99d4-b2d73bece1d5.png)

## indoor6
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785661131812-17c07dc6-33f3-446a-a16e-e5f3d2426625.png)


## indoor7
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785661165659-f47e355f-20d5-49b4-b02b-6d1293878d51.png)



## outdoor1_light
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785661213835-939870a0-bcd6-4d6f-ac8d-686e24fcb437.png)



## outdoor1
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785661258202-de92fef0-61af-414e-ae1e-816b448adf23.png)



## outdoor2
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785661338171-e2c72c8f-c3ba-48fe-a20a-4d4f1d5e9f37.png)


## outdoor3
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785661384667-4f8879cc-4621-4b14-9ac4-30e12af5e4dc.png)



## outdoor4
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665242072-4067112e-4eef-4907-8e52-20907e0de9cd.png)



## baylands
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665297900-4b30e10c-a1de-41b1-bd8e-59ff0d02cdb2.png)



## boat
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665345730-a94ab4d5-6e9e-4c8a-a125-0413e9277d7b.png)

## ego_swarm_4
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665389321-99c961c9-26e6-4f2a-9fbf-fcf934b0cd01.png)



## ego_swarm_8
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665425715-3aee5d9c-f1fd-40ad-ab56-4a065d52df9b.png)



## ego_swarm
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665448072-740df187-afbd-4838-a034-6dda3988d12f.png)



## empty
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665477810-6564dfcb-2f33-4f37-be02-b9e2876a5f74.png)

## grasping
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665513964-1b67cf6d-f33d-4b4e-a80c-84c460b55bcf.png)



## ksql_airport
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665706157-f1c5f4e5-f6d3-45c2-b7ce-9f23228ea5b1.png)



## mcmillan_airfield
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665770702-0724238a-81e8-4ed5-b517-94718a97c61f.png)



## ocean


<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665803265-568eed77-a38a-442a-a7d0-1e244850e051.png)



## robocup_indoor
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665853811-c1c49d8e-fa36-4243-b725-e61ac74886c3.png)



## sonoma_raceway
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785665925265-ff846c35-c030-47ec-b4d4-047fe033af81.png)



## warehouse
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785666151113-0a3776a1-8273-43fd-9389-fd331d44fc36.png)



## yosemite
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785666224207-0d7c9df6-2555-45d4-a0b1-62117018a635.png)



## zhihang1
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785666246215-269c7738-91b7-4941-803f-1c1654f61d60.png)

## zhihang2
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1785666280084-0c144847-e95f-452c-9677-5d29941a8789.png)
