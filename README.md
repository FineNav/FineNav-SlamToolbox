# SlamToolbox

面向 ROS2 激光点云地图的交互式处理工具，通过终端菜单完成 rosbag 录制、点云帧提取、
位姿修正、动态障碍物清除、3D 地图构建和 2D 栅格地图生成。

## 环境要求

- Ubuntu 22.04 + ROS2 Humble，或 Ubuntu 24.04 + ROS2 Jazzy（x86_64）。
- 地图数据统一存放在 `~/Map` 下。

## 安装

1. 安装 ROS2 和 pip：

   Ubuntu 22.04：

   ```bash
   sudo apt update
   sudo apt install -y ros-humble-desktop python3-pip
   ```

   Ubuntu 24.04：

   ```bash
   sudo apt update
   sudo apt install -y ros-jazzy-desktop python3-pip
   ```

   如尚未配置 ROS2 的 apt 仓库，先参考官方文档：
   [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) /
   [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)。

2. 加载 ROS2 环境并安装 slam_toolbox：

   Ubuntu 22.04：

   ```bash
   source /opt/ros/humble/setup.bash
   pip install slam_toolbox
   ```

   Ubuntu 24.04（系统 pip 受 PEP 668 保护，需加 `--break-system-packages`）：

   ```bash
   source /opt/ros/jazzy/setup.bash
   pip install --break-system-packages slam_toolbox
   ```

3. 启动：

   ```bash
   slam_toolbox
   ```

## 使用

每次打开新终端，先 `source /opt/ros/<发行版>/setup.bash` 再运行 `slam_toolbox`，
按终端菜单操作即可。地图输出到 `~/Map/<地图名称>/`，其中 `map/` 目录包含最新的
`map.pcd`（3D 地图）和 `map.pgm`、`map.yaml`（2D 栅格地图）。

## 许可证

本项目采用 [Mozilla Public License 2.0（MPL-2.0）](LICENSE) 协议开源，
版权归 FineNav 所有。每个源文件头部均带有 `SPDX-License-Identifier: MPL-2.0` 声明。

MPL-2.0 是**文件级**弱 Copyleft 协议，简单说：

- 可以自由使用、修改、分发，也允许与闭源代码组合成更大的作品并整体闭源发布；
- 但**对本项目已有源文件的修改**，必须以 MPL-2.0 向接收者公开（修改过的文件）；
- 分发时需要保留许可证与版权声明。

`docker/` 下的镜像构建脚本会引入 ERASOR2、Removert 等第三方上游项目，
它们以独立容器形式分发，遵循各自的上游许可证，详见 [NOTICE](NOTICE)。
