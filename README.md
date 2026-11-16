# Go2 爬楼建图工具集 (`爬楼建图_stair_toolkit`)

本目录包含 Go2 爬楼梯实机录制、离线 3D 台阶建图重建、离线回放脚本、核心 Python 节点与 RViz2 配置、跨电脑迁移用的便携安装包，以及已采集的爬楼数据包与实景/建图演示视频，独立归档于 `~/Go2/爬楼建图_stair_toolkit/`（对应 GitHub 仓库 `Go2-Stair-Mapping`）。

---

## 一、目录内容清单

| 文件 / 子目录 | 适用阶段 | 主要功能 / 说明 | 依赖/调用的核心资源 |
| :--- | :--- | :--- | :--- |
| `1_连接Go2并检查状态.sh` | 现场连狗 | 通过 `nmcli` 拉起 `Unitree-Go2`（有线网卡 `enp4s0`），检查 `192.168.123.x` IP 与核心话题发布状态 | `~/unitree_ros2/setup.sh` |
| `2_开启机头相机桥接.sh` | 现场连狗 | 检查有线连接后启动机头前视相机桥接节点，将 VideoHub 图像发布到 ROS 2 话题 | `./go2_front_cam_node.py` |
| `3_开启USLAM建图_可选.sh` | 现场连狗（可选） | 向 `/uslam/client_command` 发送 `mapping/start` 或 `mapping/stop` 控制狗端 USLAM 建图 | `~/unitree_ros2/setup.sh` |
| `4_录制爬楼数据包.sh` | 现场连狗 | 录制雷达、里程计、机头相机、USLAM 与狗体状态共 10 个话题到 `~/unitree_ros2/bags/` | `~/unitree_ros2/setup.sh` |
| `5_离线裁剪并重建3D台阶.sh` | 离线重建 | 自动检测或指定爬楼梯时间段并裁剪数据包，执行 4-DOF ICP 3D 配准，生成 `<原包>_3d` | `./process_stair_bag.py` |
| `6_离线GPU启动RViz2.sh` | 离线可视化 | 隔离为本机回环网卡 `lo` 并启用 NVIDIA PRIME 独显 offload 启动 RViz2（支持 `3d` / `raw`） | `./go2_stair_3d.rviz`、`./go2_slam_nav.rviz` |
| `7_离线回放数据包.sh` | 离线回放 | 在本机回环网卡 `lo` 上按 `--rate 1.0` 回放数据包（自动扫描本目录 `bags/` 与 `~/unitree_ros2/bags/`） | `./bags/`、`~/unitree_ros2/bags/` |
| `go2_front_cam_node.py` | 现场连狗 | 机头前视相机桥接节点，发布 `/go2/front_camera/image_raw/compressed` 与按需发布 `/go2/front_camera/image_raw` | `unitree_api` |
| `process_stair_bag.py` | 离线重建 | 爬楼梯数据包裁剪与 4-DOF ICP 3D 台阶重建脚本，输出 `/stair_3d/cloud_map`、`/stair_3d/scan`、`/stair_3d/odom` | `rosbag2_py`、`scipy` |
| `go2_stair_3d.rviz` | 离线可视化 | 离线 3D 重建结果默认视图配置 | RViz2 |
| `go2_slam_nav.rviz` | 离线可视化 | 原生点云 / USLAM 对比视图配置 | RViz2 |
| `bags/` | 数据存档 / 回放 | 存放已采集的典型数据包：`自软院门口`（`141M` 原始包）、`自软院门口_3d`（`186M` 3D 重建包）、`stair_demo_20261115_151447_3d`（`280M` 室内楼梯 3D 重建包） | ROS 2 `.db3` 数据包 |
| `videos/` | 演示 / 参考视频 | 存放实机建图效果与现场实景对照视频：`Go2建图.mp4`、`GO2建图右侧实景.mp4` | — |
| `go2_stair_toolkit_ubuntu2204.tar.gz` | 跨机部署 | Ubuntu 22.04 + ROS 2 Humble 通用便携安装包，用于在另一台新电脑上一键安装全套环境 | 包内含 `install.sh`、`使用说明.md` 等 |
| `README.md` | 说明文档 | 本目录的结构、日常使用流程与跨机安装包说明 | — |

> 说明：
> - `1`~`7` 脚本全部通过 `$SCRIPT_DIR` 自适应当前目录路径，独立成库，不依赖外部路径耦合；
> - `7_离线回放数据包.sh` 已适配同时检索本目录下的 `bags/` 与新录包的默认目录 `~/unitree_ros2/bags/`，直接运行即可列表选播这里的归档数据包；
> - `bags/` 中的 `.db3` 数据包因超过 GitHub 单文件 100 MB 限制未直接提交进 Git 仓库历史；从仓库 **Releases** 下载 `go2_stair_bags.tar.gz` 后，在本目录执行 `tar -xzf go2_stair_bags.tar.gz` 即可还原 `bags/`。

---

## 二、关联系统环境说明

本仓库依赖以下系统级 ROS 2 环境与默认录包目录：

- ROS 2 环境：`~/unitree_ros2/setup.sh` 与 `~/unitree_ros2/cyclonedds_ws/install/setup.bash`
- 新录包默认保存目录：`~/unitree_ros2/bags/`（脚本 `4` 写入位置；历史归档示例包存放在本目录 `./bags/` 下）

---

## 三、本机日常使用流程

以下示例默认在 `~/Go2/爬楼建图_stair_toolkit` 目录下执行（也可直接用完整路径执行）。

### 阶段 A：现场插网线连狗录包

1. **终端 1：连接有线网卡并自检**
   ```bash
   ./1_连接Go2并检查状态.sh
   ```
   确认输出中有线网卡 `enp4s0` 获取到 `192.168.123.x`，且 `/utlidar/cloud_deskewed`、`/utlidar/robot_odom`、`/sportmodestate`、`/api/videohub/response` 均有发布者。

2. **终端 2：启动机头前视相机桥接（保持运行）**
   ```bash
   ./2_开启机头相机桥接.sh
   ```

3. **（可选）开启狗端 USLAM 建图**
   ```bash
   ./3_开启USLAM建图_可选.sh          # 默认 start；结束时可传 stop
   ```
   > 仅做离线 3D 台阶重建可跳过此步；如需保留原生 USLAM 建图数据供对比则提前开启。

4. **终端 3：录制爬楼梯数据包**
   ```bash
   ./4_录制爬楼数据包.sh
   ```
   遥控 Go2 爬楼梯；爬完后在该终端按 `Ctrl+C` 停止并保存，数据包会写入 `~/unitree_ros2/bags/stair_demo_<时间戳>/`。
   - 当前录制包含的 10 个话题：`/go2/front_camera/image_raw/compressed`、`/utlidar/cloud_deskewed`、`/utlidar/robot_odom`、`/utlidar/height_map_array`、`/uslam/frontend/odom`、`/uslam/frontend/cloud_world_ds`、`/uslam/cloud_map`、`/uslam/server_log`、`/sportmodestate`、`/lowstate`。

---

### 阶段 B：拔网线离线 3D 重建与回放

此阶段全部在离线模式完成，无需连接机器狗。

5. **离线裁剪爬楼片段并重建 3D 台阶**
   ```bash
   ./5_离线裁剪并重建3D台阶.sh                      # 默认查找 ~/unitree_ros2/bags 下最新原始包
   ./5_离线裁剪并重建3D台阶.sh bags/自软院门口          # 若需对本目录归档包重跑重建，直接传入路径
   ```
   按提示回车即可使用自动检测到的推荐爬楼区间，并在原包旁生成 `<原包>_3d`。常用可选参数会直接透传给 `./process_stair_bag.py`：
   - 非交互自动裁段：`./5_离线裁剪并重建3D台阶.sh bags/自软院门口 --auto`
   - 不裁剪整包重建：`./5_离线裁剪并重建3D台阶.sh bags/自软院门口 --no-trim`
   - 保留双侧墙体不做近侧剖切：`./5_离线裁剪并重建3D台阶.sh bags/自软院门口 --no-cutaway`
   - 查看完整参数：`./5_离线裁剪并重建3D台阶.sh --help`

6. **终端 A：离线用 NVIDIA GPU 启动 RViz2**
   ```bash
   ./6_离线GPU启动RViz2.sh           # 默认加载 go2_stair_3d.rviz（重建 3D 视图）
   ./6_离线GPU启动RViz2.sh raw       # 切换为 go2_slam_nav.rviz（原生话题对比视图）
   ```

7. **终端 B：离线回放数据包**
   ```bash
   ./7_离线回放数据包.sh             # 自动列出本目录 bags/ 与 ~/unitree_ros2/bags/ 下全部数据包，回车默认播最新 [1]
   ./7_离线回放数据包.sh bags/stair_demo_20261115_151447_3d
   ```
   回放时按**空格键**可暂停/继续，**方向键**可调节倍速。`6` 与 `7` 均绑定到本机回环接口 `lo`，即使网线断开也可正常通信。

---

## 四、关于 `go2_stair_toolkit_ubuntu2204.tar.gz`（跨机部署包）

目录内的 `go2_stair_toolkit_ubuntu2204.tar.gz` 是打包好的**通用跨机移植版**，适用于部署到**另一台 Ubuntu 22.04 + ROS 2 Humble 电脑**：

- 与本目录外层 `1`~`7_*.sh` 的差异：
  - 外层 `1`~`7_*.sh` 面向本机环境定制，固定使用了网卡名 `enp4s0` 与 NVIDIA PRIME 渲染环境变量。
  - 压缩包内除了 `Go2/` 核心代码、`unitree_ros2` 消息源码（`unitree_api`、`unitree_go`、`unitree_hg`）、`使用说明.md` 和一键安装脚本 `install.sh` 外，其 `desktop_scripts/` 中的脚本内置了 `go2_nic.sh` 动态有线网卡识别与独显/默认 OpenGL 自动回退逻辑。
- 如需在其他 Ubuntu 22.04 电脑安装使用，将 `go2_stair_toolkit_ubuntu2204.tar.gz` 拷过去执行：
  ```bash
  tar -xzf go2_stair_toolkit_ubuntu2204.tar.gz
  cd go2_stair_toolkit_ubuntu2204
  chmod +x install.sh
  ./install.sh
  ```
  详细安装参数（如 `--nic <网卡名>`、`-y`、`--skip-apt`、`--skip-nm`）可查看解压后的 `使用说明.md` 或运行 `./install.sh --help`。
