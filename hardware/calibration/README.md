# 原理
imu与相机的联合标定旨在解算出imu与相机之间的相对位姿关系（平移和旋转）以及两者采样的时间差（下面统称为外参），是视觉惯性融合系统中的关键步骤。imu相机联合标定分为离线标定和在线标定，两者采用的计算模型基本一致，其中离线标定提前录制数据集，然后将数据输入到联合标定系统通过非线性最小二乘等优化算法迭代出标定参数，由于采用离线的方式进行而不用考虑实时性，因此可以对参数进行充分优化，标定结果的精度也较高。在线标定将外参作为待优化状态变量加入到整体的优化问题中，其好处是可以在系统运行过程中对外参进行调整，如果提供的初始外参不准确或者imu与相机的机械固连结构发生变化而引起外参改变，通过在线标定可以实时调整外参从而提高系统的精度和鲁棒性。通常的做法是用离线标定得到一个较准确的外参，然后将其作为具有在线标定功能的视觉惯性系统中外参的初始值。本文主要介绍离线标定相关工具的使用。

需要明白两个传感器的数学模型：相机投影模型和imu测量模型，网上资料很多也很基础，这里不作阐述，只介绍相关工具的使用。

# 相关工具介绍
## Kalibr
地址：[https://github.com/ethz-asl/kalibr](https://github.com/ethz-asl/kalibr)

eth开源并维护的相机标定库，包含单目、多目标定、imu与相机联合标定等功能，是目前用得最多的imu与相机联合标定的工具。使用kalibr对imu和相机进行联合标定前，需要先标定imu的内参和相机的内外参，其中imu需要标定的内参只包含不确定性误差，不包含确定性误差，kalibr本身不具备标定imu内参的功能，需要借助其他工具标定，而相机的标定是包含在kalibr中的，可以直接使用kalibr进行相机标定，也可用ros的camera_calibration进行标定。

## kalibr_allan
地址：[https://github.com/rpng/kalibr_allan](https://github.com/rpng/kalibr_allan)

eth开源并维护的imu内参标定库，用来对imu的不确定性误差（噪声模型）进行标定，标定的参数包含陀螺仪与加速度计的高斯白噪声，及其零偏的随机游走噪声。kalibr_allan采用matlab实现标定流程，因此要使用kalibr_allan需要安装ubuntu版本的matlab。

## imu_utils
地址：[https://github.com/gaowenliang/imu_utils](https://github.com/gaowenliang/imu_utils)

改良版本：[https://github.com/mintar/imu_utils](https://github.com/mintar/imu_utils)

与kalibr_allan一样是用于imu内参标定的工具，同样采用的allan方差法分析imu的噪声模型，好处是不需要matlab，因此用的人比较多，但是原版的imu_utils标定的结果有一定问题，详情：[https://discourse.ros.org/t/open-source-allan-variance-tool-for-rosbags/23136/4](https://discourse.ros.org/t/open-source-allan-variance-tool-for-rosbags/23136/4)，其中最主要的一点是imu_utils标定的高斯噪声密度和随机游走噪声是已经转换到离散时间下的（严格意义上来说这里的随机游走噪声已经转换为零偏不稳定性噪声，不是真正的随机游走噪声），而kalibr需要的imu噪声系数是表征在连续时间下的，因此不能直接把imu_utils标定的结果提供给kalibr。好在有大佬对imu_utils进行了改进，将其输出改为连续时间下的，本文也主要用改良后的imu_utils作为imu内参标定的工具。

## Calibration-is-All-You-Need
地址：[https://github.com/linClubs/Calibration-Is-All-You-Need?tab=readme-ov-file](https://github.com/linClubs/Calibration-Is-All-You-Need?tab=readme-ov-file)

国内作者开源的综合标定库，包含了imu、相机、激光雷达的内外参标定以及联合标定，前面提到的kalibr和imu_utils也包含在其中，并且作者提供了能运行工程下所有标定工具的Docker，使用起来非常方便，也是本文主要采用的工具。后面介绍的操作流程都是基于这个开源库，并在Docker中实现。

# 标定工具使用流程
## 构建镜像并运行容器
（前提宿主机（运行标定程序的电脑）已安装好Docker）

1.从github上clone工程到本地（后文提及的工程均指Calibration-Is-All-You-Need的主目录）：

```bash
git clone https://github.com/linClubs/Calibration-Is-All-You-Need.git
```

2.找到工程主目录下的**dockerfile**，对其中的apt镜像源进行修改（原来的已不可用），将原来的10~13行进行替换：

```dockerfile
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse" > /etc/apt/sources.list
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse" >> /etc/apt/sources.list
```

```dockerfile
RUN echo "deb http://mirrors.aliyun.com/ubuntu/ focal main restricted" > /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ focal-updates main restricted" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ focal universe" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ focal-updates universe" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ focal multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ focal-updates multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ focal-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ focal-security main restricted" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ focal-security universe" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ focal-security multiverse" >> /etc/apt/sources.list
```

3.找到工程主目录下的**docker_build.sh**，将11行改为：

```bash
sudo docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY" -v $(pwd):/calibration_ws/src/calibration --network host --name=calib_runtime calibrate:v1.0 /bin/bash
```

4.在工程主目录下，执行脚本docker_bulid.sh，开始构建镜像:

```bash
./docker_build.sh
```

构建过程中在拉取镜像的时候可能会报错：Error response from daemon: Get "[https://registry-1.docker.io/v2/"](https://registry-1.docker.io/v2/")，参考解决方案：[【已解决】docker: Error response from daemon: Get “https://registry-1.docker.io/v2/“: net/http: request c-CSDN博客](https://qiaoqiao.blog.csdn.net/article/details/144476399?spm=1001.2101.3001.6650.2&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EYuanLiJiHua%7EPosition-2-144476399-blog-143785794.235%5Ev43%5Econtrol&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EYuanLiJiHua%7EPosition-2-144476399-blog-143785794.235%5Ev43%5Econtrol&utm_relevant_index=3)简单来说就两步：

将/etc/docker/daemon.json修改为以下内容：

```bash
{
 "registry-mirrors": ["https://docker.1ms.run", "https://docker.1panel.live/"]
}

```

然后重启Docker服务：

```bash
sudo systemctl daemon-reload
sudo systemctl restart docker
```

5.构建完成后会启动并进入容器，在容器内部默认为root用户，且位于/目录下，通过ls打印当前目录，可以发现有个calibration_ws目录，这就是标定库工程目录，它是通过docker的bind mount技术将宿主机的工程目录挂载到容器的，可以理解为容器内的工程目录与宿主机的工程目录是同一个。在容器内终端使用exit退出容器后容器会终止运行，在宿主机终端执行**docker restart calib_runtime**重启容器，docker ps -a打印出正在运行的容器，可以看到calib_runtime处于运行状态，使用docker stop或宿主机关机后容器被终止运行，使用docker start calib_runtime启动容器，通过docker exec指令进入容器，简单来说：

打印当前正在运行的容器：

```bash
docker ps
```

如果没有calib_runtime，则启动：

```bash
docker start calib_runtime
```

进入容器：

```bash
docker exec -it calib_runtime bash
```

6.原工程中的imu_utils使用的是原版，由于原版的有瑕疵，咱们使用改良后的imu_utils，因此需要将改良版本添加到工程中。前面说过宿主机的工程目录与容器内的工程目录同根同源，因此本步骤中文件相关的操作在宿主机的工程目录下进行即可。

```bash
#进入imu标定工具目录
cd imu_calibration/
#clone 改良的imu_utils
git clone https://github.com/mintar/imu_utils imu_utils_modified
#修改相关文件，防止命名冲突(把imu_utils替换为imu_utils_modified)
cd imu_utils_modified/
find . -maxdepth 1 -type f -exec sed -i 's/imu_utils/imu_utils_modified/g' {} \;
cd launch/
find . -type f -name "*.launch" -exec sed -i 's/imu_utils/imu_utils_modified/g' {} \;
```

7.在容器中对工程进行编译（注意此步骤在容器中操作，而不是宿主机）:

```bash
#进入容器
docker exec -it calib_runtime bash
cd /calibration_ws/src/calibration
#在build.sh中加入编译imu_utils_modified选项
echo -e "\ncatkin build imu_utils_modified" >> build.sh
./build.sh
```

编译需要花一点时间，编译完成后即可通过docker实现所有的标定流程。

## IMU内参标定
1.录制imu数据包，将无人机上电，记得接上飞控电源，接通电源2~3分钟后，将无人机放到桌面上静止，录制过程中避免碰到无人机，录制时长大概四个小时。

```bash
rosbag record /mavros/imu/data_raw -O orinx_px4_imu.bag
```

2.进入目录/imu_calibration/imu_utils_modified/launch/，拷贝xsens.launch并重命名为px4_imu.launch，修改内容如下：

```bash
<launch>

    <node pkg="imu_utils_modified" type="imu_an" name="imu_an" output="screen">
        <param name="imu_topic" type="string" value= "/mavros/imu/data_raw"/>
        <param name="imu_name" type="string" value= "orin10"/>
        <param name="data_save_path" type="string" value= "$(find imu_utils_modified)/data/"/>
        <param name="max_time_min" type="int" value= "240"/>
        <param name="max_cluster" type="int" value= "100"/>
    </node>


</launch>

```

其中imu_name命名为无人机代号，max_time_min参数表示bag的最大时长，单位是分钟，此值需要小于bag的总时长，否则程序会一直卡住。

3.启动标定程序并播放ros数据包

```bash
roslaunch imu_utils_modified px4_imu.launch
rosbag play -r 200 orinx_px4_imu.bag
```

标定完后生成的标定文件在/imu_utils_modified/data/下，后面主要用到其中的orinx_imu_param.yaml文件，其中给出了计算出来的imu噪声和随机游走的系数值。

## 双目标定
以下操作可以自行在工程中创建一个目录，在该目录下存放标定过程所需的所有文件，比如在/kalibr目录下创建一个/orinx_calib_res目录，下面的操作都在该目录下进行。

1.准备标定板，采用april四月格，可以通过kalibr程序生成四月格pdf并打印出来

```bash
rosrun kalibr kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.038 --tspace 0.3
```

执行完后会在执行的目录生成target.pdf，打印出来即可使用。

<details class="lake-collapse"><summary id="ufbbc3594"><span class="ne-text">使用3D打印机打印标定板</span></summary><p id="u11839e80" class="ne-p"><span class="ne-text">一、先把 PDF 转成 SVG  </span></p><p id="u2ab905c8" class="ne-p"><span class="ne-text"> 推荐用 Inkscape：  </span></p><ol class="ne-ol"><li id="u7489bce4" data-lake-index-type="0"><span class="ne-text">打开 </span><strong><span class="ne-text">Inkscape</span></strong><span class="ne-text">。 </span></li><li id="u3c39d388" data-lake-index-type="0"><span class="ne-text"> 点击： 文件 → 打开</span></li></ol><ol class="ne-ol"><li id="uf92052cb" data-lake-index-type="0"><span class="ne-text"> 选择你的 AprilGrid PDF。 </span></li><li id="u53bf3fa6" data-lake-index-type="0"><span class="ne-text"> PDF 导入时选择第一页。 </span></li><li id="u2ad7b85f" data-lake-index-type="0"><span class="ne-text"> 导入后，按： Ctrl + A</span></li><li id="u8d6066c9" data-lake-index-type="0"><span class="ne-text"> 如果图形是一组对象，右键： 取消组合 / Ungroup</span></li></ol><p id="uff2d36f6" class="ne-p"><span class="ne-text">可以多点几次，直到黑色 tag 能被单独选中。</span></p><ol start="5" class="ne-ol"><li id="u2991e3e1" data-lake-index-type="0"><span class="ne-text"> 设置页面单位为 mm。 </span></li><li id="u38113252" data-lake-index-type="0"><span class="ne-text"> 确认整个黑色图案的有效尺寸。</span></li></ol><ul class="ne-ul"><li id="u08ce88e2" data-lake-index-type="0"><span class="ne-text">按 </span><code class="ne-code"><span class="ne-text">Ctrl + A</span></code><span class="ne-text"> 选中全部。 </span></li><li id="u7c7ef56a" data-lake-index-type="0"><span class="ne-text"> 顶部工具栏找到 </span><code class="ne-code"><span class="ne-text">宽</span></code><span class="ne-text"> 和 </span><code class="ne-code"><span class="ne-text">高</span></code><span class="ne-text">。 </span></li><li id="uc032bc13" data-lake-index-type="0"><span class="ne-text"> 确认单位是 </span><code class="ne-code"><span class="ne-text">mm</span></code><span class="ne-text">。 </span></li><li id="ue4e0759a" data-lake-index-type="0"><span class="ne-text"> 把宽高旁边的小锁图标锁上。 </span></li><li id="udcf18ca1" data-lake-index-type="0"><span class="ne-text"> 在 </span><code class="ne-code"><span class="ne-text">宽</span></code><span class="ne-text"> 里输入：285（如果你用 6×6、38 mm 的方案，有效 AprilGrid 应为： 285 × 285 mm）</span></li></ul><ol start="7" class="ne-ol"><li id="u01483077" data-lake-index-type="0"><span class="ne-text">页面调整到图案大小</span></li></ol><p id="ue89d27c1" class="ne-p"><span class="ne-text">选中全部黑色图案：Ctrl + A</span></p><p id="u151a85a8" class="ne-p"><span class="ne-text">然后打开文档属性：Shift + Ctrl + D</span></p><p id="u6d4c53cb" class="ne-p"><span class="ne-text">找到按钮：调整页面到所选内容</span></p><ol start="8" class="ne-ol"><li id="uaef3a2b9" data-lake-index-type="0"><span class="ne-text">居中</span></li></ol><p id="ua433ecf2" class="ne-p"><span class="ne-text"> 如果页面是 295 × 295 mm，图案是 285 × 285 mm，那么每边留 5 mm。  </span></p><ul class="ne-ul"><li id="ub948791d" data-lake-index-type="0"><span class="ne-text">选中全部黑色图案： Ctrl + A</span></li><li id="u75de9701" data-lake-index-type="0"><span class="ne-text"> 对象 → 组合  </span></li><li id="u8b9f99eb" data-lake-index-type="0"><span class="ne-text">打开对齐面板： 对象 → 对齐和分布</span></li><li id="ub82d08a5" data-lake-index-type="0"><span class="ne-text">在对齐面板里找到： 相对于：</span></li><li id="u23a9df33" data-lake-index-type="0"><span class="ne-text">选择：页面</span></li></ul><ul class="ne-ul"><li id="u6b372fe3" data-lake-index-type="0"><span class="ne-text">点击两个按钮： </span></li></ul><p id="ud32d4462" class="ne-p"><span class="ne-text">水平居中</span></p><p id="u30508d9f" class="ne-p"><span class="ne-text">垂直居中</span></p><ol start="9" class="ne-ol"><li id="u5cd95681" data-lake-index-type="0"><span class="ne-text"> 保存为： 文件 → 另存为 → Plain SVG / 普通 SVG</span></li></ol><p id="u92270a8b" class="ne-p"><span class="ne-text">文件名例如：aprilgrid_6x6_38mm.svg</span></p><p id="ub7feed5c" class="ne-p"><span class="ne-text"></span></p><p id="ue905f748" class="ne-p"><span class="ne-text">二、 在 Bambu Studio 中做白色底板  </span></p><ol class="ne-ol"><li id="uebd8e283" data-lake-index-type="0"><span class="ne-text">打开 </span><strong><span class="ne-text">Bambu Studio</span></strong><span class="ne-text">。 </span></li><li id="u06b65c76" data-lake-index-type="0"><span class="ne-text"> 选择打印机： Bambu Lab H2D</span></li><li id="uaa383885" data-lake-index-type="0"><span class="ne-text"> 添加一个 Cube： Add Primitive → Cube</span></li><li id="ufb804ab6" data-lake-index-type="0"><span class="ne-text"> 设置尺寸： </span></li></ol><p id="u2a24f658" class="ne-p"><span class="ne-text">X = 295 mm</span></p><p id="uf9eefef0" class="ne-p"><span class="ne-text">Y = 295 mm</span></p><p id="ucd17e1a2" class="ne-p"><span class="ne-text">Z = 2.0 mm</span></p><ol start="5" class="ne-ol"><li id="ub00f99e0" data-lake-index-type="0"><span class="ne-text"> 选中底板，设置耗材颜色为： 白色 PLA</span></li></ol><p id="ucf131a59" class="ne-p"><br></p><p id="ufb4c6e7b" class="ne-p"><span class="ne-text">三、 导入 AprilGrid SVG到bambu studio</span></p><p id="uede73f85" class="ne-p"><span class="ne-text"></span></p><p id="ua8cceef4" class="ne-p"><span class="ne-text">四、在bambu studio中添加白色基板</span></p><p id="ua5286f80" class="ne-p"><span class="ne-text"></span></p><p id="u0bbc8e9d" class="ne-p"><span class="ne-text">五、切片打印</span></p><p id="uc5894698" class="ne-p"><span class="ne-text">测量出是：</span></p><p id="ub18bbbf0" class="ne-p"><span class="ne-text">tagSize=0.034244m</span></p><p id="u2557b550" class="ne-p"><span class="ne-text">b=10.217mm</span></p><p id="u4fbac006" class="ne-p"><span class="ne-text">tagSpacing=0.2983588</span></p></details>
2.创建标定板配置信息文件april_6x6.yaml，内容如下：

```bash
target_type: 'aprilgrid' # gridtype   标定板类型
tagCols: 6               # number of apriltags  大黑快数目
tagRows: 6               # number of apriltags
tagSize: 0.038          # size of apriltag, edge to edge [m]  单位m
tagSpacing: 0.3          # ratio of space between tags to tagSize
```

注意tagSize不是生成四月格时指定的0.088m，因为实际打印出来测量结果为0.0243m，相关参数图示说明如下：

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2025/png/827790/1737554824895-84902e19-502c-46c4-85dc-69ece33b26ff.png)

3.录制双目数据集，将标定板平整地贴在白色背景的地方，为了减少数据量，需要对图像数据进行降频，使用ros的throttle实现：

```bash
rosrun topic_tools throttle messages /camera/infra1/image_rect_raw 4.0 /camera/infra1/image_rect_raw_downsample
rosrun topic_tools throttle messages /camera/infra2/image_rect_raw 4.0 /camera/infra2/image_rect_raw_downsample
```

注意降频后的图像数据话题名为：/camera/infra1/image_rect_raw_downsample和/camera/infra2/image_rect_raw_downsample。

对将降频后的图像数据进行录制：

```bash
rosbag record /camera/infra1/image_rect_raw_downsample /camera/infra2/image_rect_raw_downsample -O orinx_stereo_downsample_calib.bag
```

录制过程中标定板不要离开相机视野范围，开始和结束要平稳进行，尽量使标定板出现在视野所有角落。可打开rqt_image_view对录制的图像进行观察。

4.执行双目标定程序：

```bash
rosrun kalibr kalibr_calibrate_cameras --target april_6x6.yaml --bag orinx_stereo_downsample_calib.bag --models pinhole-radtan pinhole-radtan --topic /camera/infra1/image_rect_raw_downsample /camera/infra2/image_rect_raw_downsample --bag-from-to 7 105
```

注意修改--bag-from-to，表示要使用数据时间段的起始时间和结束时间，单位：秒(s)，这个参数可以剔除掉刚开始录制和结束时一些出入视野等画面。

执行完后会生成三个文件：

cam-report-cam.pdf 包含绘制的图片和标定的参数。

cam-results-cam.txt 以文本文件储存的标定结果。

cam-camchain.yaml 以YAML格式储存的标定结果。它可以直接用来作为相机-IMU校正的输入。

## IMU与相机联合标定
执行目录同样在双目标定时所在的目录下。

1.录制双目和imu的数据集，录制的图像数据集仍然是降频后的，录包采集数据的起始和结束阶段注意别晃动太大，如从桌子上拿起或者放下。如果有这样的动作，在标定阶段应该跳过bag数据集的首尾的数据。采集数据的时候应该给imu各个轴足够的激励，如先依次绕各个轴运动，运动完后来个在空中画8字之类的操作，当然也要注意别运动太剧烈，图像都模糊了。



```bash
rosbag record /camera/infra1/image_rect_raw_downsample /camera/infra2/image_rect_raw_downsample /mavros/imu/data_raw  -O orinx_imu_stereo_downsample_calib.bag
```

2.准备配置文件，包括imu内参信息文件orinx_imu_param_kalibr.yaml（通过imu_utils_modified标定得到），双目标定信息配置文件camchain-orinx_stereo_downsample_calib.yaml（kalibr_calibrate_cameras标定得到），其中imu的配置文件需要按照kalibr要求的格式填写，创建imu内参信息文件orinx_imu_param_kalibr.yaml，根据imu_utils_modified标定得到的orinx_imu_param.yaml中的内容进行填写，参考如下：

```bash
#Accelerometers
accelerometer_noise_density: 1.9672627825158434e-03   #Noise density (continuous-time)
accelerometer_random_walk:   2.5367471808698973e-04   #Bias random walk

#Gyroscopes
gyroscope_noise_density:     4.7137630579032418e-04   #Noise density (continuous-time)
gyroscope_random_walk:       1.9902100767849608e-06   #Bias random walk

rostopic:                    /mavros/imu/data_raw      #the IMU ROS topic
update_rate:                 250.0      #Hz (for discretization of the values above)
```

双目的配置文件直接用kalibr标定双目的输出文件。

3.执行标定程序：

```bash
rosrun kalibr kalibr_calibrate_imu_camera --target april_6x6.yaml --bag orinx_imu_stereo_downsample_calib.bag --bag-from-to 6 142 --cam camchain-orinx_stereo_downsample_calib.yaml --imu orinx_imu_param_kalibr.yaml --imu-models scale-misalignment --timeoffset-padding 0.01
```

注意修改--bag-from-to，表示要使用数据时间段的起始时间和结束时间，单位：秒(s)，这个参数可以剔除掉刚开始录制和结束时一些出入视野等画面。

标定过程很慢，标定完后会输出以下文件：

report-cam-％BAGNAME％.pdf：以PDF格式报告。包含所有用于文档的图。 

results-cam-％BAGNAME％.txt：结果摘要为文本文件。

camchain-％BAGNAME％.yaml：结果为YAML格式。该文件可用作相机imu校准器的输入。



## 标定结构同步到Vins-Fusion
### 外参
\\wsl.localhost\Ubuntu-22.04\home\zhangrun\tools\Calibration-Is-All-You-Need\kalibr\orin_10_v3_calib_res\results-imucam-orinx_imu_stereo_downsample_calib.txt

```bash
Transformation (cam0):
-----------------------
T_ci:  (imu0 to cam0): 
[[ 0.00852045 -0.99995604 -0.00391508  0.02770411]
 [ 0.0229631   0.00410985 -0.99972787  0.01948947]
 [ 0.9997      0.00842823  0.02299711 -0.05175067]
 [ 0.          0.          0.          1.        ]]

T_ic:  (cam0 to imu0): 
[[ 0.00852045  0.0229631   0.9997      0.05105155]
 [-0.99995604  0.00410985  0.00842823  0.02805896]
 [-0.00391508 -0.99972787  0.02299711  0.02078274]
 [ 0.          0.          0.          1.        ]]

timeshift cam0 to imu0: [s] (t_imu = t_cam + shift)
-0.002596592642307161


Transformation (cam1):
-----------------------
T_ci:  (imu0 to cam1): 
[[ 0.00905962 -0.9999524  -0.00362097 -0.02222186]
 [ 0.0254823   0.00385081 -0.99966786  0.01947286]
 [ 0.99963422  0.00896434  0.02551597 -0.05204311]
 [ 0.          0.          0.          1.        ]]

T_ic:  (cam1 to imu0): 
[[ 0.00905962  0.0254823   0.99963422  0.05172918]
 [-0.9999524   0.00385081  0.00896434 -0.02182925]
 [-0.00362097 -0.99966786  0.02551597  0.02071386]
 [ 0.          0.          0.          1.        ]]

timeshift cam1 to imu0: [s] (t_imu = t_cam + shift)
-0.002675034273277853
```

要的是cam0 to imu0和cam1 to imu0，填写到Fast-Drone-XI35/src/realflight_modules/VINS-Fusion-gpu/config/fast_drone_250.yaml、

```bash
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 0.00852045,  0.0229631,   0.9997,      0.05105155,
            -0.99995604,  0.00410985,  0.00842823,  0.02805896,
            -0.00391508, -0.99972787,  0.02299711,  0.02078274,
            0.          , 0.          , 0.          , 1.        ]

body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 0.00905962,  0.0254823,   0.99963422,  0.05172918,
            -0.9999524,   0.00385081,  0.00896434, -0.02182925,
            -0.00362097, -0.99966786,  0.02551597,  0.02071386,
            0.          , 0.          , 0.          , 1.        ]

td: -0.002596592642307161                           # -0.001 initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)
```

### 内参
```bash
cam0
-----
  Camera model: pinhole
  Focal length: [383.7598888764043, 384.8925538781987]       #fx,fy
  Principal point: [322.8198727696575, 237.94749700020026]   #cx,cy
  Distortion model: radtan
  Distortion coefficients: [-0.003006530398038659, -0.0019414681222989217, 0.0008514068817637035, 0.0006444348796559118]   #k1,k2,p1,p2
  Type: aprilgrid
  Tags: 
    Rows: 6
    Cols: 6
    Size: 0.034244 [m]
    Spacing 0.010216998747199998 [m]

cam1
-----
  Camera model: pinhole
  Focal length: [382.9464520416595, 384.09539614429093]			#fx,fy
  Principal point: [322.5467809298544, 237.6637220880597]		#cx,cy
  Distortion model: radtan
  Distortion coefficients: [-0.0016124188794249222, -0.006258620277501587, 0.0006825571185065275, 0.0003767815568973883]		#k1,k2,p1,p2
  Type: aprilgrid
  Tags: 
    Rows: 6
    Cols: 6
    Size: 0.034244 [m]
    Spacing 0.010216998747199998 [m]
```

填写到/root/Fast-Drone-XI35/src/realflight_modules/VINS-Fusion-gpu/config/left.yaml和/root/Fast-Drone-XI35/src/realflight_modules/VINS-Fusion-gpu/config/right.yaml

