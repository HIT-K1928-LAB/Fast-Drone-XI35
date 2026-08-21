# Docker 使用

## RK3588 + openEuler

RK3588 上建议让 openEuler 作为宿主系统，容器使用 Ubuntu 20.04 ARM64
用户态。这样可以直接使用 ROS Noetic 的官方二进制包，同时复用宿主机的
Rockchip 内核、RKNPU、RGA 和 MPP 驱动。容器基础系统不需要与宿主系统相同。

`Dockerfile.rk3588` 与现有镜像的主要差异：

- CUDA、cuDNN 和 TensorRT 被移除；OpenCV 4.5.4/3.4.16 使用 CPU + NEON。
- 安装 RKNN Toolkit Lite2 与 RKNN C Runtime 2.3.2。
- librealsense 使用 RSUSB 后端从源码构建，不依赖 Ubuntu DKMS 内核模块。
- 保留 ROS Noetic、MAVROS、Sophus、Ceres、LCM 和自定义 `cv_bridge_454`。

### 宿主机检查

```bash
uname -m
docker version
readlink -f /sys/class/drm/renderD129/device/driver
```

RK3588 应输出 `aarch64`，且最后一个命令应指向 `RKNPU` 驱动。不同板卡镜像的
设备编号可能不同；当前 openEuler 镜像使用 `/dev/dri/renderD129`。

### 构建

在 RK3588 板端执行原生 ARM64 构建：

```bash
cd Fast-Drone-XI35/docker/Dockerfile
make rk3588 USE_PROXY=false SPEED=8
```

需要代理时沿用 PC/Jetson 的参数：

```bash
make rk3588 USE_PROXY=true CONTAINER_HTTP_PROXY=http://192.168.31.2:7897 \
  CONTAINER_HTTPS_PROXY=http://192.168.31.2:7897
```

也可以在已经配置 ARM64 binfmt/QEMU 的 x86 服务器上交叉构建。先确认模拟器
可执行 ARM64 容器：

```bash
docker run --rm --platform linux/arm64 alpine uname -m
```

输出 `aarch64` 后，在服务器上的工程目录执行：

```bash
cd Fast-Drone-XI35/docker/Dockerfile
make rk3588 USE_PROXY=true SPEED=24 \
  RK3588_PLATFORM=linux/arm64 RK3588_DOCKER_BUILDKIT=1
docker image inspect fastdronexi35:rk3588 --format '{{.Architecture}}'
```

本项目已在 `k1928-c` 上按此方式构建并在 RK3588 上验证。QEMU 编译大型 C++
依赖仍比原生编译慢，但可使用服务器的 CPU、内存和 Docker 缓存。`k1928-a`、
`k1928-d` 或本机若未注册 ARM64 binfmt，会报 `exec format error`。

构建完成后可通过镜像仓库，或用 SSH 流式传到板端：

```bash
ssh k1928-c 'docker save fastdronexi35:rk3588 | gzip -1' | \
  ssh rk3588-01 'docker load'
```

镜像较大且链路延迟高时，使用内网镜像仓库或并行分片传输会更快。没有可用的
ARM64 模拟器时，使用前面的 RK3588 原生构建命令。

### 启动与 NPU 验证

```bash
cd Fast-Drone-XI35/docker/Dockerfile
./container_run_rk3588.sh
docker exec -it fd_runtime_rk3588 bash
```

启动脚本使用 `--privileged` 并映射 `/dev`，以透传 NPU、RGA、MPP、USB 相机和
串口。RK3588 默认使用扩展口 UART7（`/dev/ttyS7:921600`）连接 PX4；可在创建
容器时通过 `MAVROS_FCU_URL` 覆盖，例如：

```bash
MAVROS_FCU_URL=/dev/ttyUSB0:921600 ./container_run_rk3588.sh
```

部署稳定后可以按实际设备收紧权限。

### MID360 与海康 USB3 工业相机

已在 `rk3588-01` 上验证以下实机连接：MID360 接到专用 `eth0`，海康
`MV-CS020-10UC`（序列号 `DA1135025`）通过 USB3 Vision 接入。容器启动脚本已
使用 `--network host` 和 `/dev` 透传，因此不需要额外的 Docker 网络或 USB 映射。

首次在宿主机配置 MID360 专用网口。此操作只修改 `eth0`，不会影响 SSH 使用的
`rename5`：

```bash
sudo nmcli connection modify "Wired connection 1" \
  connection.interface-name eth0 ipv4.method manual \
  ipv4.addresses 192.168.1.5/24 ipv4.gateway "" ipv6.method disabled
sudo nmcli connection up "Wired connection 1"
```

本机 MID360 的实际地址为 `192.168.1.103`，数据端口为 `56300`（点云）和
`56400`（IMU）。该地址已写入
`src/localization/FAST-LIVO2/config/mid360_rk3588.json`；若日后在 Livox Viewer
中改了雷达 IP，必须同步修改该文件里的 `lidar_configs[0].ip`。重新构建或首次
使用工程时执行：

```bash
cd /root/Fast-Drone-XI35
./tools/make.sh fastlivo -c
source devel/setup.bash
```

`fastlivo` 组会先构建 `livox_ros_driver2`，并自动为其生成 ROS1 的包清单；不应
从当前工程路径直接执行上游的 `livox_ros_driver2/build.sh`，该脚本假定驱动直接
位于工作区 `src/` 下。

启动并验收 MID360：

```bash
# 终端 1：MID360；应出现 /livox/lidar 和 /livox/imu
roslaunch fast_livo mid360_rk3588.launch

# 终端 2：确认 MID360 数据持续发布
rostopic hz /livox/lidar
rostopic hz /livox/imu
```

镜像内置 `Livox-SDK2`、`livox_ros_driver2` 所需依赖，以及 ARM64 可用的
Aravis/GStreamer 诊断工具。已验证相机能以标准 USB3 Vision/GenICam 协议被枚举，
控制面可读取 `1624x1240`、`BayerRG8`、连续触发等参数：

```bash
arv-tool-0.6 -n Hikrobot-DA1135025 control \
  DeviceVendorName DeviceModelName DeviceSerialNumber Width Height PixelFormat
```

本板的通用 Aravis 0.6 数据流测试会在开始取流后报 `Internal data stream error`，
故它仅作为枚举/控制诊断工具，不能替代正式相机驱动。已用海康 MVS 5.0.2 ARM64
官方 Python 示例验证 `DA1135025` 能连续抓取 1624×1240 图像帧。MVS 安装在 RK3588
宿主机的 `/opt/MVS`，运行脚本会把它以只读方式挂载到容器；MVS 安装包和 `/opt/MVS`
均不提交到仓库。

海康 USB3 相机需要较大的 usbfs URB 内存上限。RK3588 宿主机（不是容器）使用以下
服务将其持久设为 `2000 MB`。首次安装时立即生效，以后开机时会在 Docker 启动前
生效：

```bash
cd /home/rk3588/Fast-Drone-XI35
sudo install -m 0644 \
  docker/systemd/fastdrone-rk3588-usbfs-memory.service \
  /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now fastdrone-rk3588-usbfs-memory.service
systemctl show fastdrone-rk3588-usbfs-memory.service \
  -p LoadState -p ActiveState -p FragmentPath
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

最后一条命令应输出 `2000`。该数值是可分配上限，不会在开机时立即预留 2 GB。

启动 ROS 图像采集（默认使用本机相机序列号）：

```bash
roslaunch fast_livo hikrobot_cs020_rk3588.launch
# 另一个终端：应持续显示频率并收到 bayer_rggb8 原始图像
rostopic hz /hikrobot_camera/image_raw
rostopic echo -n 1 /hikrobot_camera/image_raw/header
```

话题为 `/hikrobot_camera/image_raw`，编码为 `bayer_rggb8`，保留传感器原始数据。
更换相机时可指定序列号，例如
`roslaunch fast_livo hikrobot_cs020_rk3588.launch device_serial:=<序列号>`。如需彩色
图像，可用 `image_proc` 的 debayer 节点订阅该原始话题。运行时只保留一个应用独占相机。

相机默认使用固定曝光 `20000 us`、增益 `6 dB`，适合作为 5 Hz 室内低照度的起点。
若画面仍暗，优先逐步提高曝光（例如 `25000`、`30000 us`），再提高增益；增益过高会
明显增加噪声。示例：

```bash
roslaunch fast_livo hikrobot_cs020_rk3588.launch exposure_time_us:=30000 gain_db:=6
```

在 5 Hz 下不要把曝光长时间设得高于约 `200000 us`，否则会降低实际帧率；动态建图时
应明显低于该上限，以减少运动模糊。
将 `exposure_time_us:=0 gain_db:=-1` 可保留相机内部已有的手动配置。
可用 `exposure_auto:=Continuous` 开启连续自动曝光，或用 `Once` 仅自动调一次；完成
现场亮度调试后，建议读出合适的曝光值并恢复 `exposure_auto:=Off` 的固定曝光。
该相机固件未暴露自动曝光目标灰度和上下限节点；若硬件自动曝光对场景变化不敏感，可用
节点侧的可预测模式：`exposure_auto:=Software`。它每 0.5 秒根据原始 Bayer 图像的平均
亮度调节曝光，默认目标灰度为 100、范围为 1000--90000 us。
有订阅者时，相机节点每 0.5 秒读取并发布当前 SDK 实际使用的数值（自动曝光时会
变化）；无人订阅时不访问这些控制寄存器，以减少 USB 控制传输对图像流的干扰：

```bash
rostopic echo /hikrobot_camera/exposure_time_us
rostopic echo /hikrobot_camera/gain_db
```

当前海康启动文件还会显式设置标定用图像几何：`1624x1240`、`OffsetX=0`、
`OffsetY=0`、`Binning=1x1`，节点不进行软件裁剪，将 SDK 图像缓存设为 5，并将
采集帧率设为 `5 Hz`。发生 USB 数据流恢复、设备帧号回退时，节点会先释放当前
buffer，再停止采集、重建缓存并重新写入和回读相机配置。镜头焦距和对焦位置是
机械状态，必须在完成标定后保持不动。恢复次数达到
`recovery_warning_threshold`（默认 60 秒内 10 次）时会持续输出链路不稳定告警，
但不会主动退出相机节点；如果 SDK 的停止、配置或重新启动操作本身失败，节点仍会
退出并保留明确的错误码，避免在未知采集状态下继续发布数据。启动文件默认会在 3 秒后
重新拉起异常退出的相机进程；调试时可用 `respawn:=false` 关闭自动重启。

ROC-RK3588S-PC 的 USB-A 口使用 `fcd00000` xHCI/USB3 PHY。满载实测中，usbmon 在
相机图像流端点 `EP3 IN` 捕获到 `-EPROTO`（USB 协议/响应错误）和 `-EPIPE`
（端点 stall），它们发生在 MVS `Transfer Stall` 之前；因此这是 USB 事务层故障，
不是 ROS 话题、图像频率或 FAST-LIVO2 队列产生的错误。FAST-LIVO2 会显著提高该错误
的发生概率，但单独持续订阅图像不会。驱动在复制完图像后会立即归还 MVS buffer，
并允许通过 `usb_transfer_size_bytes`、`usb_transfer_ways` 调节 U3V 传输；本机测试
`2 ways` 或 `2 MiB` 传输块均未根治，所以默认保留 SDK 常用的 `1 MiB / 8 ways`。

当前相机声明最大取电 `800 mA`，接近 USB3 Type-A 的标准供电上限。若仍频繁出现
`Transfer Stall`，先使用带独立供电的 USB3 Hub 做供电/重定时 A/B；其次可通过
USB-C OTG 主机口转接，测试另一组控制器/PHY。若有源 Hub 后稳定，优先检查开发板
USB VBUS 和整机电源余量；若 USB-C 稳定而 USB-A 仍失败，优先检查 USB-A 口及其
`fcd00000` PHY/BSP；两端口均失败但同一相机在 x86 主机稳定时，再升级或向板卡厂商
反馈 RK3588S 的 xHCI/PHY BSP。

RK3588 的 FAST-LIVO2 配置还在接收端启用了基于图像时间戳的 `5 Hz` 软件保护；
它在图像颜色转换前丢弃超频帧，并将 ROS 订阅队列及解码后的图像队列限制为 3 帧。
相关参数位于 `mid360_hikrobot_rk3588.yaml` 的 `image_input` 段，可按数据源调整；
将 `max_rate_hz` 或 `buffer_max_frames` 设为 `0` 可分别关闭对应保护。

`mid360_hikrobot_rk3588.yaml` 和 `camera_hikrobot_cs020.yaml` 已写入当前这套
MID360--海康相机组合的实测外参和内参。更换相机、雷达、镜头或改变安装位置后，必须
重新标定并更新对应参数，不能直接沿用当前数值。

### 工程编译边界

当前工程中的以下包直接使用 NVIDIA CUDA/TensorRT，不能仅靠换 Dockerfile 在
RK3588 上编译或运行：

- `yolo_trt_detector`
- `yopo_planner`
- `superpoint`
- `local_sensing_node`、`sensor_simulator`
- 仿真目录中的 `darknet_ros` CUDA 配置

这些节点需要分别改成 RKNN C API/Lite2（推理）或 CPU/OpenCL 实现。其余 ROS
包可先用 `catkin_tools` 跳过上述包进行板端编译；完整功能迁移还需要修改节点
代码和把 ONNX 模型转换成 `target_platform=rk3588` 的 `.rknn` 模型。模型转换
通常放在开发机完成，板端只安装 RKNN Runtime。

## nvidia jetson平台

1. 构建 Jetson 镜像

```bash
cd Docker/Dockerfile
make jetson
```

Jetson 镜像直接包含运行环境和容器初始化脚本，不再单独拆分基础镜像。

2. 容器启动

`container_run.sh` 会自动把当前宿主机工程目录挂载到容器的 `/root/Fast-Drone-XI35`，不再使用 `Fast-Drone-XI35` named volume。脚本默认开启 X11 转发，保留镜像 entrypoint，所以容器启动时仍会执行 `container_init.sh` 来配置 SSH、LCM、MAVROS 和相机服务。

```bash
### 首次容器启动
./container_run.sh
### 后续容器启动
docker start fd_runtime
docker exec -it fd_runtime bash
```

可按需覆盖运行参数：

```bash
CONTAINER_NAME=fd_runtime_jetson IMAGE_NAME=fastdronexi35:orin SHM_SIZE=16g ./container_run.sh
PROJECT_DIR=/home/your_account/code/Fast-Drone-XI35 ./container_run.sh
ENABLE_X11=false ./container_run.sh
```

3. 编译工程

容器启动时不会自动执行 `catkin_make`。进入容器后按需手动编译：

```bash
cd /root/Fast-Drone-XI35
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 代理配置

Docker 镜像构建默认启用代理：

```bash
USE_PROXY=true
PROXY_PORT=7897
```

默认代理地址会根据环境自动选择：

- WSL + Docker Desktop：`http://host.docker.internal:7897`
- 普通 Ubuntu/Linux：`http://127.0.0.1:7897`

这是因为在 Docker 构建容器内部，`127.0.0.1` 指向容器自身，不一定是宿主机代理；WSL + Docker Desktop 场景下需要使用 `host.docker.internal` 才能让构建容器访问 Windows 上的代理软件。

如果不需要代理：

```bash
make pc USE_PROXY=false
make jetson USE_PROXY=false
```

如果代理端口不是 `7897`：

```bash
make pc PROXY_PORT=7890
```

如果需要手动指定完整代理地址：

```bash
make pc CONTAINER_HTTP_PROXY=http://192.168.31.6:7897 CONTAINER_HTTPS_PROXY=http://192.168.31.6:7897
```

Jetson 构建同样支持这些参数：

```bash
make jetson PROXY_PORT=7897
make jetson CONTAINER_HTTP_PROXY=http://127.0.0.1:7897 CONTAINER_HTTPS_PROXY=http://127.0.0.1:7897
```

注意：这里的代理参数主要用于 Dockerfile 构建过程中的 `git clone`、`wget`、`apt`、`pip` 等命令。Docker 自己拉取基础镜像时，仍需要宿主机 Docker daemon / Docker Desktop 能访问对应镜像仓库；如果基础镜像拉取失败，需要另外配置 Docker daemon 代理或 registry mirror。

## x86 平台

⚠️ 前置要求

1. 宿主机必须正确安装好Nvidia驱动，使用nvidia-smi测试输出，CUDA Version字段必须大于等于11.8

2. 宿主机必须正确安装好Nvidia Docker Toolkit
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

3. 确认电脑的CUDA_ARCH_BIN，对Dockerfile.pc的ARG CUDA_ARCH_BIN=7.5（默认）进行修改

```bash
nvidia-smi --query-gpu=compute_cap --format=csv,noheader,nounits
```

4. 由于需要从 Docker Hub pull 一个 base 镜像，需要提前解决 Docker daemon 的代理或镜像源问题，否则会出错。上面的 `USE_PROXY` / `CONTAINER_HTTP_PROXY` 只影响 Dockerfile 内部下载依赖，不等同于 Docker daemon 拉镜像的代理配置

参考：

```bash
vim /etc/docker/daemon.json
###
# 添加以下内容
{
 "registry-mirrors": ["https://docker.1ms.run", "https://docker.1panel.live/"]
}
###
sudo systemctl daemon-reload
sudo systemctl restart docker
```

5. 下载TensorRT到Docker文件夹下

下载地址（需要登录）：
https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/secure/8.6.1/tars/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz

6. 构建Docker镜像

```bash
cd Docker/Dockerfile
make pc
```

`make pc` 会先检查上一级目录是否存在 TensorRT 安装包：

```bash
../TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz
```

如果文件不存在，脚本会退出。由于 NVIDIA 下载页面需要登录，TensorRT 安装包需要手动下载后放到 `Docker/` 目录下。

构建默认使用代理。如果需要关闭代理：

```bash
make pc USE_PROXY=false
```

构建完成后

```bash
docker image ls
```

输出

```bash
REPOSITORY      TAG IMAGE ID        CREATED         SIZE
fastdronexi35   pc  a615436b212d    19 hours ago    24.4GB
```

7. 实例化Docker容器

其中$TARGET_DIR需要替换为宿主机的代码目录，或者其他的想要映射到容器内的目录

- -v $TARGET_DIR:/root/Fast-Drone-XI35 作用是将宿主机的$TARGET_DIR目录映射到容器内的/root/Fast-Drone-XI35，实现宿主机和容器共同使用同一份文件。两个系统均可对文件进行修改。


```bash
xhost +local:root # 允许本地 root 显示 X 窗口

docker run -it \
--name fd_runtime_pc \
--runtime=nvidia \
--gpus all \
--net=host \
--privileged \
-e DISPLAY=$DISPLAY \
-e QT_X11_NO_MITSHM=1 \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v $TARGET_DIR:/root/Fast-Drone-XI35 \
fastdronexi35:pc \
bash
```

8. 测试容器

- 进入容器终端

```bash
docker exec -it fd_runtime_pc bash
```

- 建议将代码source setup.bash放入.bashrc环境变量中,同时配置ROS_LOG_DIR的环境变量，根据自己实际的容器中代码路径下述命令

```bash
echo 'export ROS_LOG_DIR=/root/Fast-Drone-XI35/log' >> /root/.bashrc
echo "source /root/Fast-Drone-XI35/devel/setup.bash" >> /root/.bashrc
```

- 运行VINS-Fusion测试

```bash
# 在代码根目录进行编译
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
# source环境变量(如果已经添加过环境变量，则跳过)
source devel/setup.bash
# 启动各个节点
roslaunch imu_filter imu_filter.launch
roslaunch superpoint superpoint_frontend.launch
roslaunch vins fast_drone_250.launch
roslaunch vins rviz.launch
# 播放数据集
rosbag play your_dataset.bag
```

⚠️ 注意第一次运行前端superpoint_frontend.launch会卡住，是正常现象，内部在进行跨平台的.onnx文件构建，等待3~4分钟即可正常运行，下一次也可正常启动。

### gazebo仿真镜像

在fastdrone_xi35:pc镜像存在的基础下，在Dockerfile文件夹下 make pc_sim
