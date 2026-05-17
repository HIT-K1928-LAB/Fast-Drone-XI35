# Docker 使用

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
git clone https://github.com/NVIDIA-AI-IOT/deepstream_tlt_apps.git
cd deepstream_tlt_apps/TRT-OSS/x86
nvcc deviceQuery.cpp -o deviceQuery
./deviceQuery
### 输出
Detected 1 CUDA Capable device(s)

Device 0: "NVIDIA GeForce GTX 1660 SUPER"
  CUDA Driver Version / Runtime Version          12.2 / 11.8
  CUDA Capability Major/Minor version number:    7.5

CUDA Capability Major/Minor version number这个字段的数字就是CUDA_ARCH_BIN
###
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


1. 下载TensorRT到Docker文件夹下

下载地址（需要登录）：
https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/secure/8.6.1/tars/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz

2. 构建Docker镜像

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

3. 实例化Docker容器

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

4. 测试容器

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
