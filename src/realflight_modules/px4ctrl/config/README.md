# px4ctrl Runtime Configs

这个目录放实机运行时控制参数。除 `template/` 外，这里的配置文件默认由 Git 忽略，用来保存每架无人机自己的质量、控制器、PID 和推力相关参数。

首次在一架新飞机上部署时，从模板复制一份本机配置：

```bash
cp -n template/ctrl_param_fpv.yaml ctrl_param_fpv.yaml
```

然后只编辑复制出来的 `ctrl_param_fpv.yaml`。`template/` 只保留字段结构和默认参考值，不要把真实飞机的控制参数写回模板。
