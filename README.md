# PG49 UWB Ranging Driver

English | [简体中文](README.zh-CN.md)

This driver reads range measurements from a PG49 module over a serial port and publishes them on `/uwb/distance_stamped` (`pg49_ranging/DistanceStamped`) in ROS 1 Noetic.

The `distance` field is in meters. The `header.stamp` field records the host's ROS time when the serial read completes. Frames completed in the same read share a timestamp. The optional `t1–t6` values reported by the module are internal UWB ranging timestamps based on a separate clock.

## Build

On a Linux system with ROS Noetic and Catkin installed, clone this repository into the workspace's `src` directory.

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/pg49_ws/src
cd ~/pg49_ws/src
git clone https://github.com/js-ch3n/uwb_pg_49_driver.git
```

From the workspace root, build the driver and the bundled `serial` library.

```bash
cd ~/pg49_ws
catkin_make
source devel/setup.bash
```

## Launch

Connect the PG49 module and run the following command. The default serial port is `/dev/ttyUSB0`, and the default baud rate is `115200`.

```bash
roslaunch pg49_ranging uwb_driver.launch
```

If your module uses a different serial port, specify it at launch.

```bash
roslaunch pg49_ranging uwb_driver.launch port:=/dev/ttyUSB1
```

If opening the serial port fails with a permission error on Ubuntu, run `sudo usermod -aG dialout "$USER"`, then log out and back in.
