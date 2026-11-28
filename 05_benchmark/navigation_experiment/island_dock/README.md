# 岛台北侧绕箱到站 · Pre 主场景

Stretch 3 从 RoboCasa 厨房的原出生点出发，向北侧绕过固定箱体，再回到原红杯任务使用的底盘停靠位。Pre 展示底盘转向、绕箱、恢复目标航向和停稳。评测终点是停稳后的底盘。

| 项目 | 配置 |
|---|---|
| 厨房 | `seed=7`、`layout=1`、`style=0` |
| 出生位姿 | `(2.20, -1.30, 0)`（米、弧度） |
| A 工位 | `(3.55, -1.30, 0)`（米、弧度） |
| 箱体 | 中心 `(2.88, -1.81)`，长宽高 `0.24 × 0.18 × 0.38 m` |
| 规划净空 | 收臂底盘半径 `0.43 m`，规划另加 `0.05 m` |

[独立配置](../../../01_navigation/maps/island_dock_experiment/profile.json) 将箱体矩形同时标成禁行区。直行中心线距箱体最近边 `0.42 m`，低于 `0.48 m` 规划净空。A* 生成的折返点为 `(2.975, -1.175)`，向北偏 `0.125 m`。A 带目标平面位置和航向，停靠由常规路径跟踪完成。

## 运行和现场展示

先按[项目安装及资产说明](../../../README.md#快速运行)准备环境。在仓库根目录运行：

```bash
.venv/bin/python 03_data_generate/build_scene.py
.venv/bin/python 03_data_generate/build_navigation_experiment.py \
  --profile 01_navigation/maps/island_dock_experiment/profile.json

.venv/bin/python 05_benchmark/evaluate_navigation_prototype.py \
  --profile 01_navigation/maps/island_dock_experiment/profile.json \
  --station A --no-cameras --out out/pre-island-dock
.venv/bin/python 05_benchmark/plot_navigation_experiment.py --run-dir out/pre-island-dock
```

`truth.csv`、`trajectory.csv` 和 `metrics.json` 保存在输出目录。绘图命令生成轨迹图、误差时间图和统计表。复跑时请使用新的输出目录。评测命令加入 `--cruise-speed 0.22` 可复现原巡航设置的速度对照。

现场运行下列命令，打开 [本地 Web Console](http://127.0.0.1:8773/)，选择「工位 A」并开始导航。在 MuJoCo 查看器里选择 `navigation_overview` 相机可以俯视箱体和走廊。

```bash
.venv/bin/python 01_navigation/run_console.py \
  --profile 01_navigation/maps/island_dock_experiment/profile.json --viewer --port 8773
```

## 地图采集

配置随附这版场景的稀疏占据地图、PGM 和 ROS 地图 YAML，共记录 5,498 个栅格。测绘阶段使用碰撞几何为测绘航点规划，使用轮角、陀螺仪和 360° 激光扫描定姿、更新占据栅格。下面的命令可以独立重采，产物写到 `out/`。

```bash
.venv/bin/python 01_navigation/run_navigation.py --headless --planner grid \
  --scene scenes/kitchen/generated/kitchen_island_dock_experiment.xml \
  --localization slam --initial-pose 2.20 -1.30 0 \
  --goal 3.55 -1.30 0 --goal 4.55 -1.30 3.141592653589793 \
  --goal 2.20 -1.30 3.141592653589793 --goal 2.20 -1.30 0 --timeout 310 \
  --map-out out/pre-island-survey/grids.json --export-map out/pre-island-survey/kitchen
```

## 比较口径

Mini SLAM 驾驶一趟。轮式里程计、陀螺仪融合及扫描匹配后的估计消费同一批传感器数据，逐点比较 MuJoCo `base_link` 世界位姿。位置误差取平面欧氏距离，航向误差用归约到 ±π 的角差。RMSE 和终点定位误差分别对比估计与真值，停车误差对比停稳真值与目标。 [差速模型及融合公式](../README.md#比较口径)

## 实测结果

采用 `0.30 m/s` 巡航指令上限时，从准备完成到停稳耗时 **42.60 秒仿真时间**，较 `0.22 m/s` 对照的 **52.90 秒** 缩短约 **19.5%**。A 工位实际停车误差为 **5.53 cm / 1.63°**，853 个 20 Hz 控制采样点未见机器人与非地面物体接触。

| 定位方法 | 平面 RMSE | 航向 RMSE | 终点定位误差 | 终点航向误差 |
|---|---:|---:|---:|---:|
| 轮式里程计 | 2.54 cm | 0.89° | 3.43 cm | 0.95° |
| 融合里程计 | 2.34 cm | 0.26° | 3.11 cm | 0.34° |
| Mini SLAM | 3.77 cm | 0.26° | 3.26 cm | 0.34° |

短路线内里程计漂移较小。扫描匹配接受了两次平移校正，第一次发生在 `1.05 s`，使误差曲线出现跳变，第二次在 `27.70 s` 降低了当时的误差。融合里程计在这趟行驶中有最低的位置 RMSE。西侧的较长路线可用来对比扫描匹配抑制累计漂移的表现。 [西侧通道实验](../README.md#实测结果)

[轨迹对比图](trajectories.png) · [定位误差时间图](localization_errors.png) · [评测数值](metrics.json)
