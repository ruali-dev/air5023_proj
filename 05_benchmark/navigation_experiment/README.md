# 西侧通道导航定位实验

Stretch 3 从 RoboCasa 厨房西侧通道北端出发，绕过一个固定箱体，再在 A 工位面对西墙停稳。直行、转向、绕行和激光墙距停靠在同一趟运行中完成。B 是北端返回工位。

| 项目 | 配置（米、弧度） |
|---|---|
| 厨房 | `seed=7`、`layout=1`、`style=0` |
| 出生位姿 | `(0.50, -1.40, -π/2)` |
| 障碍 | 中心 `(0.65, -2.70)`，长宽高 `0.32 × 0.36 × 0.38` |
| A 停靠位姿 | `(0.10, -3.65, π)`，距西墙 `1.10 m` |
| B 停靠位姿 | `(0.10, -1.50, π)`，距西墙 `1.10 m` |
| 规划净空 | 收臂底盘半径 `0.43 m`，规划另加 `0.05 m` |

[独立 profile](../../01_navigation/maps/patrol_experiment/profile.json) 记录场景文件、已知出生位姿、目标位姿和箱体范围。生成器用原厨房 XML 生成单独的 `kitchen_navigation_experiment.xml`。箱体侵入从出生点到 A 预停靠点直行所需的底盘净空，A* 在固定图上规划经箱体西侧的路径。箱体边界同时是显式禁行区。

## 复现与展示

先按[项目安装及资产说明](../../README.md#快速运行)准备环境。在仓库根目录运行：

```bash
.venv/bin/python 03_data_generate/build_scene.py
.venv/bin/python 03_data_generate/build_navigation_experiment.py

.venv/bin/python 05_benchmark/evaluate_navigation_prototype.py \
  --profile 01_navigation/maps/patrol_experiment/profile.json \
  --station A --no-cameras --out out/navigation-pre
.venv/bin/python 05_benchmark/plot_navigation_experiment.py --run-dir out/navigation-pre
```

重新评测时指定一个新的输出目录。`truth.csv` 和 `trajectory.csv` 以仿真时间逐行同步，`metrics.json` 记录到站真值误差、匹配统计和控制采样点的接触记录。`plots/` 下生成轨迹图、误差图和 Markdown 数表。对照原巡航速度时给同一条评测命令加 `--cruise-speed 0.22`，用独立输出目录。

现场演示可以启动原 Web Console，选择「工位 A」后开始导航。在 MuJoCo 查看器里选择 `navigation_overview` 相机，可以俯视西侧通道。

```bash
.venv/bin/python 01_navigation/run_console.py \
  --profile 01_navigation/maps/patrol_experiment/profile.json --viewer --port 8765
```

控制台地址为 [127.0.0.1:8765](http://127.0.0.1:8765/)。

### 重采传感器地图

仓库随配置提供采集的 `grids.json`、PGM 和 ROS 地图 YAML。地图来自一趟八航点测绘行驶，共记录 9,119 个栅格。测绘阶段用碰撞几何栅格规划已知航点，使用轮角、底盘陀螺仪与 360° LiDAR 定位和更新栅格。下面的命令将新地图输出到 `out/`。

```bash
.venv/bin/python 01_navigation/run_navigation.py --headless --planner grid \
  --scene scenes/kitchen/generated/kitchen_navigation_experiment.xml \
  --localization slam --initial-pose 0.50 -1.40 -1.5707963267948966 \
  --goal -0.24 -2.18 -1.5707963267948966 --goal -0.24 -3.30 -1.5707963267948966 \
  --goal 0.10 -4.10 3.141592653589793 --goal 0.70 -4.07 0 \
  --goal -0.23 -3.40 1.5707963267948966 --goal -0.23 -2.05 1.5707963267948966 \
  --goal 0.10 -1.50 3.141592653589793 --goal 0.50 -1.40 0 --timeout 440 \
  --map-out out/navigation-pre-survey/grids.json --export-map out/navigation-pre-survey/kitchen
```

## 比较口径

三种估计从同一个已知出生位姿初始化。轮式里程计使用左右轮角增量，融合里程计加入陀螺仪航向增量，Mini SLAM 在融合估计上加固定地图的激光扫描匹配。评测由 Mini SLAM 提供反馈，只驾驶一次，三个估计共享同一组传感器数据和时间戳。MuJoCo `base_link` 世界位姿用作评测真值。

差速模型用 `Δs = r(ΔφR + ΔφL) / 2` 和 `Δθwheel = r(ΔφR − ΔφL) / b` 计算每次运动增量。`r` 是轮半径，`b` 是轮距。融合使用 `Δθfused = 0.15 Δθwheel + 0.85 Δθgyro`。Mini SLAM 在这个预测位姿附近搜索平移和旋转，以扫描端点和占据图的一致性评分做校正。

平面误差为估计点与真值点的欧氏距离。航向误差为估计减真值后归约到 ±π 的最小有符号角差，两者分别按 20 Hz 控制采样计算 RMSE。终点定位误差比较估计和真值，实际停车误差比较真值和 A 工位目标位姿。

## 实测结果

单趟到 A，2,308 个同步采样，18 次扫描校正通过接受条件。20 Hz 控制采样未记录到机器人与非地面物体接触。

| 定位方法 | 平面 RMSE | 航向 RMSE | 终点定位误差 | 终点航向误差 |
|---|---:|---:|---:|---:|
| 轮式里程计 | 10.52 cm | 5.10° | 12.06 cm | 8.31° |
| 融合里程计 | 7.80 cm | 1.58° | 9.83 cm | 2.22° |
| Mini SLAM | 5.58 cm | 0.27° | 5.95 cm | 0.07° |

A 工位**实际停车误差为 4.51 cm / 1.26°**。误差图可以看到扫描校正时估计位置有短时跳变。这次行驶中，Mini SLAM 的平面误差和航向误差在整段 RMSE 上均低于另外两种方法。另一次同会话 A→B 验证中，B 也成功停稳，实际停车误差为 6.49 cm / 0.86°。

该实验启用 `0.30 m/s` 巡航指令上限。在距离航点 `0.35 m` 至 `0.10 m` 的范围内逐步恢复原减速曲线。对比同场景的 `0.22 m/s` 设置，起步前准备完成至停车的仿真用时从 **128.90 s** 缩短至 **115.35 s**。两组的角速度限制和墙距停靠控制相同。

[轨迹对比图](trajectories.png) · [定位误差时间图](localization_errors.png) · [评测数值](metrics.json)
