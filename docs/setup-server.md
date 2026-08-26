# CARLA 0.10.0 服务器环境

## 路径边界

- CARLA 发行包：`/mnt/fast18/sunbo/CARLA-0.10.0/Carla-0.10.0-Linux-Shipping`
- 研究仓库：`/mnt/fast18/sunbo/carla`
- Conda 环境：`carla10`
- Python：3.10

发行包和研究代码必须分开。Git 仓库不保存 CARLA 二进制、引擎资源或 Python wheel。

## 安装版本匹配的 Python API

```bash
conda activate carla10
cd /mnt/fast18/sunbo/CARLA-0.10.0/Carla-0.10.0-Linux-Shipping

python -m pip install \
  ./PythonAPI/carla/dist/carla-0.10.0-cp310-cp310-linux_x86_64.whl
```

验证：

```bash
python -c "import carla; print(carla.__file__)"
```

## 启动仿真器

通过图形桌面观察时：

```bash
cd /mnt/fast18/sunbo/CARLA-0.10.0/Carla-0.10.0-Linux-Shipping
LANG=C.UTF-8 LC_ALL=C.UTF-8 ./CarlaUnreal.sh
```

仅在远程终端运行时：

```bash
LANG=C.UTF-8 LC_ALL=C.UTF-8 ./CarlaUnreal.sh -RenderOffScreen -nosound
```

启动前关闭其他占用 CARLA 2000 端口的仿真程序。

## 同步研究仓库后

```bash
cd /mnt/fast18/sunbo/carla
conda activate carla10
python -m pip install -r requirements.txt
```

先执行离线测试，再运行后续实现的冒烟入口。真实仿真脚本必须输出即时进度，并在退出时
只清理由本次运行创建的 Actor。
