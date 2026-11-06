# SlamToolbox

面向 ROS2 激光点云地图的交互式处理工具，通过终端菜单完成地图数据录制、点云帧提取、回环检测修正、动态障碍物清除、3D 地图构建和 2D 栅格地图生成。

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

## Interactive SLAM GUI 显示

Interactive SLAM 通过 Docker 容器 + X11 转发显示界面，两种环境均可使用：

- **WSL2**：需 Windows 11 的 WSLg（`WSL2_GUI_APPS_ENABLED=1`、`DISPLAY=:0`），
  或自行安装 VcXsrv 等 X server 并正确设置 `DISPLAY`。WSL 下自动使用 Mesa
  软件渲染，无需 NVIDIA 容器运行时。
- **Ubuntu 桌面**：在 X11 / Wayland 桌面会话中运行即可。若主机有 NVIDIA GPU 且
  已安装 `nvidia-container-toolkit`，会自动启用 GPU 直通；否则回退到
  `/dev/dri` 或软件渲染。

如自动选择有误，可用 `SLAM_TOOLBOX_GPU_MODE=gpu|dri|software` 强制指定渲染后端。

### HiDPI 界面缩放

容器里的 Dear ImGui 使用固定 13px 字体，不跟随桌面的 DPI 缩放，高分辨率屏上
界面组件会偏小。启动 GUI 时会自动读取宿主机的 `Xft.dpi` 计算缩放系数
（范围 1.0–2.5），通过在容器内编译并 `LD_PRELOAD` 一个小型垫片放大字体与控件，
无需修改镜像。可用 `SLAM_TOOLBOX_UI_SCALE` 手动指定系数，设为 `1` 关闭缩放：

```bash
SLAM_TOOLBOX_UI_SCALE=2 slam_toolbox
```

## 许可证

本项目采用 [Mozilla Public License 2.0（MPL-2.0）](LICENSE) 协议开源。

ERASOR2、Removert 等第三方上游项目，以独立容器形式分发，遵循各自的上游许可证，详见 [NOTICE](NOTICE)。
