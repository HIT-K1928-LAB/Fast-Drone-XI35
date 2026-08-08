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
