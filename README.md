# PG49 UWB 测距驱动

适用于 ROS 1 Noetic。驱动通过串口读取 PG49 模块的测距数据，并发布到 `/uwb/distance_stamped`（`pg49_ranging/DistanceStamped`）。
`distance` 为距离（米），`header.stamp` 为主机完成本批串口读取时的 ROS 时间。
同一批读取中的多帧共用这个时间戳。设备可选输出的 `t1–t6` 是 UWB 芯片内部的测距计数，与 ROS 时间使用不同的时钟基准。

## 编译

在已安装 ROS Noetic 和 Catkin 的 Linux 环境中，将仓库克隆到工作空间的 `src` 目录。

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/pg49_ws/src
cd ~/pg49_ws/src
git clone https://github.com/js-ch3n/uwb_pg_49_driver.git
```

回到工作空间根目录，编译驱动和仓库自带的 `serial` 库。

```bash
cd ~/pg49_ws
catkin_make
source devel/setup.bash
```

## 启动

连接 PG49 模块后，运行以下命令。默认串口为 `/dev/ttyUSB0`，波特率为 `115200`。

```bash
roslaunch pg49_ranging uwb_driver.launch
```

如果串口编号不同，可以在启动时指定。

```bash
roslaunch pg49_ranging uwb_driver.launch port:=/dev/ttyUSB1
```

如果打开串口时提示权限不足，在 Ubuntu 上可运行 `sudo usermod -aG dialout "$USER"`，然后退出登录并重新登录。
