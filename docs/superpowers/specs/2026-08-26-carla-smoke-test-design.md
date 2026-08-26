# CARLA 连接冒烟测试设计

## 目标

为 `D:\Work\carla` 增加第一个真实 CARLA 入口：

```bash
python -m src.smoke --config cfg/simulator.yaml
```

该命令只验证 CARLA 0.10.0 Python 客户端能否连接正在运行的服务器并读取世界摘要。
它不加载地图、不修改世界设置、不生成 Actor，也不销毁任何 Actor。

## 成功输出

成功时按固定顺序输出：

```text
连接 CARLA：localhost:2000，超时 10.0s
连接成功
客户端版本：0.10.0
服务器版本：0.10.0
地图：Carla/Maps/Town10HD_Opt
同步模式：false
车辆：0
行人：0
Actor 总数：...
请求耗时：...s
```

实际版本和地图名称由服务器返回。请求耗时使用单调时钟计算，只用于观察连接状态，
不作为性能基准。

## 命令行接口

`src/smoke.py` 提供：

```python
def build_parser() -> argparse.ArgumentParser: ...
def main(argv: Sequence[str] | None = None) -> int: ...
```

参数：

- `--config PATH`：配置文件路径，默认 `cfg/simulator.yaml`。

进程退出码：

- `0`：成功连接并读取完整摘要。
- `2`：配置文件无效或缺少必需字段。
- `3`：CARLA 包不可导入、连接失败、超时或服务器读取失败。
- `4`：客户端和服务器主次版本不同。

入口必须捕获已知边界错误并输出单行 `错误：...`，不向普通用户打印 Python traceback。

## 配置

沿用 `cfg/simulator.yaml`：

```yaml
client:
  host: localhost
  port: 2000
  timeout_seconds: 10
```

`host` 必须是非空字符串；`port` 必须为 `1..65535` 的整数；`timeout_seconds` 必须是
大于零的有限数值。`world` 和 `random_seed` 可继续存在，但冒烟测试不使用它们改变服务器。

## 模块边界

### `src/carla_experiments/client.py`

定义不可变摘要：

```python
@dataclass(frozen=True)
class WorldSummary:
    client_version: str
    server_version: str
    map_name: str
    synchronous_mode: bool
    vehicle_count: int
    walker_count: int
    actor_count: int
    elapsed_seconds: float
```

定义连接函数：

```python
def inspect_world(
    carla_module: Any,
    host: str,
    port: int,
    timeout_seconds: float,
    clock: Callable[[], float] = time.monotonic,
) -> WorldSummary: ...
```

函数建立 `carla.Client`、设置超时、读取版本、world、map、settings 和 actors，并统计：

- `vehicle.*`
- `walker.pedestrian.*`
- 全部 Actor

函数不调用 `load_world()`、`reload_world()`、`apply_settings()`、`spawn_actor()`、
`destroy()` 或 `world.tick()`。

### `src/carla_experiments/config.py`

保留现有 `load_config()`，新增：

```python
@dataclass(frozen=True)
class ClientConfig:
    host: str
    port: int
    timeout_seconds: float

def parse_client_config(config: Mapping[str, Any]) -> ClientConfig: ...
```

所有配置错误统一抛出 `ValueError`，错误信息点明字段名称。

### `src/carla_experiments/progress.py`

复用现有 `ProgressReporter`，确保远程终端立即看到连接阶段和结果。

## 版本规则

客户端与服务器完整版本字符串允许补丁信息不同，但点号分隔的前两个数字必须一致。例如：

- `0.10.0` 与 `0.10.0`：通过。
- `0.10.0-dirty` 与 `0.10.1`：通过。
- `0.9.16` 与 `0.10.0`：退出码 `4`。

无法解析前两个数字时按连接读取失败处理，退出码 `3`。

## 错误处理

- `carla` 使用延迟导入，使本机没有 CARLA wheel 时仍能收集和运行离线测试。
- 导入失败时提示用户在服务器安装发行包自带的 `cp310` wheel。
- CARLA RPC 异常保留异常类型和简短消息，例如 `RuntimeError: time-out of 10000ms`。
- 配置在导入 CARLA 前验证，配置错误不应被误报为服务器错误。
- 任何失败都不执行清理，因为本命令从不创建或修改服务器资源。

## 测试

本机测试不连接 CARLA，使用假模块、假 Client、假 World 和假 ActorCollection 验证：

1. 客户端接收正确的 host、port 和 timeout。
2. 摘要字段和 Actor 数量正确。
3. 计时使用注入时钟，结果可重复。
4. 测试双对象未暴露任何修改世界的方法调用。
5. 配置边界验证覆盖空 host、非法端口和非正超时。
6. CLI 成功时退出 `0` 并输出所有字段。
7. 配置错误、导入错误、RPC 错误和版本不匹配使用各自退出码。
8. 现有配置、进度和布局测试继续通过。

验证命令：

```bash
python -m pytest -v
python -m ruff check .
python -m compileall -q src tests
```

## 服务器验收

用户手动同步后，在 CARLA 图形窗口仍运行的情况下执行：

```bash
cd /mnt/fast18/sunbo/carla
conda activate carla10
python -m src.smoke --config cfg/simulator.yaml
```

验收条件：退出码为 `0`，服务器版本为 `0.10.x`，地图为当前实际地图，Actor 数量与
服务器状态一致；运行前后地图、同步设置和 Actor 列表均未被该脚本改变。

## 范围之外

本功能不启动 CARLA、不自动切换地图、不生成交通、不推进同步帧、不创建相机、不保存
图像，也不判断平台是否适合最终研究。这些工作按研究路线在后续独立功能中实现。
