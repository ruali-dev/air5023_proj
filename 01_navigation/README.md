# 01 · Navigation Prototype

这个原型将已有的 Mini 2D SLAM、A*、底盘跟踪和 Web Console 接入 Stretch 的 MuJoCo 仿真。底盘导航和巡检可在项目的 RoboCasa 厨房中独立运行。总分工见 [协作手册](../docs/development/README.md)。

[西侧通道导航定位实验](../05_benchmark/navigation_experiment/README.md) 提供独立绕障场景、采集地图、同轨迹定位对比和 Pre 复现命令。

## 实时 Web Console

按项目 [安装说明](../README.md#快速运行)生成厨房场景后，在项目根目录运行：

```bash
.venv/bin/python 01_navigation/run_console.py --viewer --capture-dir out/navigation-console
```

打开 `http://127.0.0.1:8765/`。点击地图选点或选 A/B 工位，再点「开始导航」。网页显示估计位姿、规划路径、实际估计轨迹、仿真时钟和最近一次到达的头部 RGB。加 `--viewer` 可同时用 MuJoCo 窗口观察同一台机器人。省略 `--viewer` 只运行网页端，`--no-cameras` 可在无可用图形后端的环境中跳过 RGB。

待命、行驶、取消停稳和下一次下发目标都在同一幕中进行。地图和定位持续共用，浏览器刷新也可以读回当前任务。「重置仿真」会回到场景出生位姿并重新初始化定位。服务仅绑定回环地址，面向单人本机调试。

默认使用 [厨房配置](maps/kitchen/profile.json)，其 A、B 分别位于东侧通道的北端和南端。A* 以固定占据图规划，预留 Stretch 收臂时的底盘净空，对岛台、吧凳和台面等 LiDAR 切面覆盖不全的区域额外设禁行区。工位先对准预停靠点，最后约 0.35 m 沿工位航向直线进入。LiDAR 用墙面距离修正进场深度，停车后再次核对墙距，缺少可靠回波时停止。

```bash
# 同一幕内验证 A → B → A，取消第一次 B 请求、停稳后重试
.venv/bin/python 05_benchmark/evaluate_navigation_prototype.py --out out/kitchen-patrol
```

评测写出各站 RGB、`metrics.json`、轮式里程计 / 融合里程计 / SLAM 的 `trajectory.csv`，以及独立的 `truth.csv`。真值仅在评测端读取。运行中，目标、控制台位姿、跟踪和到达判断都使用固定图上的传感器估计及工位激光距离。地图默认匹配仓库 seed 7、layout 1、style 0 的场景。改动场景后可使用 `--profile` 指向匹配的新地图和工位配置。

锁定版本的 Stretch 和 RoboCasa 厨房中，A → B → A 在同一幕跑完，B 的首次请求在行驶中取消并停稳，随后重试到达。三次停车位置误差为 6.49、5.49、7.32 cm，航向误差为 0.25°、2.88°、0.98°，末端墙距残差约 1.8 cm。20 Hz 控制采样中未见机器人与非地面物体接触。轮式里程计、融合里程计和 Mini SLAM 的平面位置 RMSE 分别为 18.36、7.41 和 6.39 cm。停靠深度已用墙距闭环校正，横向精度仍受地图定位误差影响。

## 现有文件和运行

| 文件 | 当前能力 |
|---|---|
| [navigation/navigation_interface.py](navigation/navigation_interface.py) | 已知空闲通道内 waypoint 闭环、超时停止 |
| [run_navigation.py](run_navigation.py) | 独立导航入口，支持预设航点、栅格规划和 Mini SLAM 定位 |
| [navigation/grid_navigation.py](navigation/grid_navigation.py) | 用静态栅格规划并跟踪路线，支持连续巡检点 |
| [navigation/slam_navigation.py](navigation/slam_navigation.py) | 接入 Stretch 传感器，保存占用栅格和加载固定地图 |
| [run_console.py](run_console.py) | 开启连续仿真、实时地图操作和工位停靠 |

在项目根目录运行：

```bash
.venv/bin/python 01_navigation/run_navigation.py --headless
# GUI 查看底盘运动
.venv/bin/python 01_navigation/run_navigation.py
```

## 静态栅格导航与巡检

显式加上 `--planner grid` 启用移植的 A* 和路径跟踪基线。省略目标时，使用厨房场景的默认航点。

```bash
.venv/bin/python 01_navigation/run_navigation.py --headless --planner grid
```

重复指定 `--goal X Y YAW` 可按顺序巡检。坐标在 MuJoCo 世界系，位置单位为米，航向为弧度。加上 `--capture-dir` 后，每次停车都保存一张头部 RGB，并在 JSON 输出中列出路径、实际停车位姿、仿真时间和图片路径。抓取任务仍使用原有航点接口。

```bash
.venv/bin/python 01_navigation/run_navigation.py --planner grid \
  --goal 3.55 -1.30 0 --goal 2.20 -1.30 0 \
  --timeout 150 --capture-dir out/navigation-inspection
```

栅格以 5 cm 分辨率投影 MuJoCo 碰撞几何，覆盖起点和目标附近的区域。净空检查使用机器人当前碰撞外形的包络圆，导航期间机械臂保持固定。可以直达时使用直线，否则用八邻域 A* 规划。底盘闭环使用 `RobotInterface.get_state()` 提供的仿真真值位姿，以 `move_base(v,w)` 执行，遇到不可达、净空不足或超时会发送零速。

基线适用于平坦静态场景。碰撞包围盒可能让窄通道变得不可达，地图在任务开始时生成一次。动态避障和自动重规划待后续接入。

导航回归测试覆盖直行、绕障、堵塞、转向停车和超时。锁定版本的官方 Stretch 模型在简化 MuJoCo 场景中通过了直行、绕障和堵塞停车，绕行到点误差为 2.94 cm / 1.97°。

```bash
.venv/bin/python -m pytest -q 05_benchmark/tests/test_navigation_core.py 05_benchmark/tests/test_grid_navigation.py
```

## Stretch Mini SLAM

加上 `--localization slam` 后，底盘跟踪使用轮式里程计、底盘 IMU 的陀螺仪和 360° LiDAR 扫描匹配估计位姿，同时构建 5 cm 占用栅格。定位从场景出生位姿起步，也可以用 `--initial-pose X Y YAW` 指定已知起点。目标和地图共用该世界坐标系。

首次采图时，以碰撞几何栅格规划已知路线，必要时用重复的 `--goal` 覆盖巡检区域。停车后将激光采集的地图保存为 `grids.json`。

```bash
.venv/bin/python 01_navigation/run_navigation.py --headless --planner grid \
  --localization slam --map-out out/stretch-slam/grids.json \
  --export-map out/stretch-slam/kitchen
```

固定地图巡检时，A* 和扫描匹配都使用保存的地图。未观测或证据不足的栅格视为未知。地图需要覆盖起点、目标和途经通道，SLAM 适用于已知起点附近的局部定位。CLI 的 `visits[].pose` 是估计的停车位姿，`slam` 字段记录扫描匹配统计。

```bash
.venv/bin/python 01_navigation/run_navigation.py --headless --planner grid \
  --localization slam --map-in out/stretch-slam/grids.json
```

激光地图反映底盘 LiDAR 高度的切面。低矮障碍、悬空物体和场景变动仍需其他感知或人工约束。相关回归测试覆盖轮角顺序、IMU 坐标变换、激光外参、地图读写和固定地图反馈。

`--export-map` 同时导出三值 PGM 和 ROS 地图 YAML。稀疏 `grids.json` 保留匹配所需的 log odds，供 A* 和 Mini SLAM 读取。简化场景中，固定地图巡检的平面位置 RMSE 为 4.0 cm，两站停车误差为 5.4 cm / 2.7° 和 2.0 cm / 2.1°。

默认厨房地图包含 10,479 个观测栅格，使用 Stretch 的轮角、底盘陀螺仪和激光扫描采集。参考地图生成命令如下。三值 PGM 的原点与稀疏图一样使用 MuJoCo 世界坐标，图像第一行对应最大 Y。

```bash
.venv/bin/python 01_navigation/run_navigation.py --headless --planner grid --localization slam \
  --goal 3.55 -1.25 0 --goal 4.72 -1.25 -1.5707963267948966 \
  --goal 4.72 -4.40 3.141592653589793 --goal 0.1 -4.40 1.5707963267948966 \
  --goal 0.1 -1.50 -1.5707963267948966 --goal 0.1 -4.40 0 \
  --goal 4.72 -4.40 1.5707963267948966 --goal 4.72 -1.25 3.141592653589793 \
  --goal 2.2 -1.30 0 --timeout 850 \
  --map-out out/kitchen-survey/grids.json --export-map out/kitchen-survey/kitchen
```

```bash
.venv/bin/python -m pytest -q 05_benchmark/tests/test_stretch_slam.py 05_benchmark/tests/test_grid_navigation.py
```

## 后续协作

Mini SLAM、建图定位评测和感知研究由原负责人继续维护。组员可以在现有导航基线上改进路径跟踪、动态避障和重规划，验证净空及禁行区，联调停靠与机械臂，补充不同路线和起点的回归。Web Console 可作为公共调试工具。改进建议和协作可以通过 Issue / PR 提出。

## 交接和验收

06 组提供视觉目标，02 组调用本组导航；07 组提供 `RobotInterface.move_base(v,w)`。底盘速度单位为 m/s、rad/s；本组不直接修改机械臂目标。

原航点入口 `navigate(robot, waypoints, timeout=45)` 仍用于红杯任务。巡检入口记录各站到达位姿和失败原因，可用评测脚本核对净空、终点精度和停车行为。
