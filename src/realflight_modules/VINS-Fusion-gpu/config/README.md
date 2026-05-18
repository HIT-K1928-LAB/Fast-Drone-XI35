# VINS-Fusion Runtime Configs

这个目录放实机运行时配置。除 `template/` 外，这里的配置文件默认由 Git 忽略，用来保存每架无人机自己的双目、IMU、外参、内参、曝光和 RViz 配置。

首次在一架新飞机上部署时，从模板复制一份本机配置：

```bash
cp -n template/* .
```

然后只编辑复制出来的文件，例如：

- `fast_drone_250.yaml`
- `left.yaml`
- `right.yaml`
- `mono_downward.yaml`
- `xi35_mono_downward.yaml`
- `realsense_bind_exposure.json`
- `vins_rviz_config.rviz`

`template/` 只保留字段结构和默认参考值。不要把真实飞机的标定参数、外参、内参或场地参数写回模板。`realsense_bind_exposure.json` 是严格 JSON，不能写注释，使用前也需要先复制到当前目录再改。
