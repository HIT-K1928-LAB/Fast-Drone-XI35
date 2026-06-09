# 飞控烧录bootloader和固件

### 编译
```bash
git clone git@github.com:HIT-K1928-LAB/PX4-Autopilot.git   --recursive

cd PX4-Autopilot/
# 安装依赖
bash ./Tools/setup/ubuntu.sh

# 编译bootloader
make hkust_nxt-dual_bootloader

# 编译飞控固件
make hkust_nxt-dual_default
```

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1776735088787-2ed60c0b-27f4-43a4-911a-3b0d32d9b98a.png)

/PX4-Autopilot/build/hkust_nxt-dual_bootloader/hkust_nxt-dual_bootloader.elf

PX4-Autopilot/build/hkust_nxt-dual_default/hkust_nxt-dual_default.px4

把上述两个文件下载到本地

### 烧录bootloader
+ 安装zadig（安装DFU驱动）

[https://zadig.akeo.ie/#](https://zadig.akeo.ie/#)

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1776734423937-d1ac9d6f-d4dc-46e1-aef1-de692a5c3bb2.png)

插入进入DFU模式（按住飞控上的按钮插入Type-C上电）的飞控板，选择DFU的这个设备，进行upgrade Driver

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1776734511716-df792cf9-da6c-4fff-903c-9f2f81e21f41.png)

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1776734614979-1047e2ba-6482-42ed-aefb-f287beeb2b82.png)

+ 安装<font style="color:rgb(63, 74, 84);">ST的官方烧录工具，用于飞控的Bootloader烧录</font>

[https://www.st.com.cn/zh/development-tools/stm32cubeprog.html#get-software](https://www.st.com.cn/zh/development-tools/stm32cubeprog.html#get-software)

+ 安装QGround<font style="color:rgb(63, 74, 84);">（PX4飞控的地面站软件）</font>

（官网）[http://qgroundcontrol.com/](http://qgroundcontrol.com/)

（发布的下载地址）[https://github.com/mavlink/qgroundcontrol/releases](https://github.com/mavlink/qgroundcontrol/releases)



+ 按住飞控上按钮，上电，进入DFU模式

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2025/png/8427069/1765801066584-4cde62b5-e3a8-48ea-8788-18ab2205bef3.png)

打开STM32CubeProgrammer，连接进行烧录hkust_nxt-dual_bootloader.elf文件

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2025/png/8427069/1765801132099-ae0de793-a5d3-4ad3-bbb0-db78646bca88.png)

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1776735687286-16786fe2-aa7b-4c89-8ddd-5938863a5a0b.png)





### 烧录固件
打开QGround，重新连接飞控，可以正常识别到飞控

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1776735802428-c087d0bb-5374-49ff-b2af-16a0bb348e35.png)

选择自定义固件文件

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1776735838622-a82a0727-1904-4cd1-90b7-1a7f65ec9277.png)

选择刚刚下载到本地的hkust_nxt-dual_default.px4固件，完成烧录

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/8427069/1776735900988-8e6b1703-9e18-40d3-908b-f125300e7bf2.png)



# 飞控配置






