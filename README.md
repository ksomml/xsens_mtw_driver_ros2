# Xsens MTw Awinda driver for ROS2
```
                   !!!  WORK IN PROGRESS !!!
        Any helpful pull requests are really appreciated!
```
This project contains a ROS2 driver for the Xsens MTw Awinda system sensors.

<img src="xsens_mtw_awinda_system.jpg" alt="Image Description" width="250" height="250">


## TODO

- Implement recording option which saves formatted log file
- Better Code
- Implement services (get_ready(), status(), record(), etc.)
- Remove debugging logs
- Update README
- More fixes (correct shutdown/cleanup, etc.)
- params.yaml


## Hardware

- Xsens MTw Awinda System


## Software

- The driver is developed upon the XDA 2022, with ROS2 on Ubuntu 22.04 LTS
    - XDA 2022 =/= public XDA 2022 (the public XDA 2022 only supports MTi devices)
- ISO C++17 Standard corrections have been applied to the XDA 2022 files
- Instead of using the Xsens Quaternion, this driver uses the state-of-the-art quaternion filter [VQF](https://doi.org/10.1016/j.inffus.2022.10.014)


## Prerequisites

- [Ubuntu Linux](https://www.releases.ubuntu.com/)  (tested with 22.04)
- [ROS2](https://docs.ros.org/) (tested with Humble)


## Usage

`TODO`

Commands:

- `ros2 run xsens_mtw_driver_ros2 xsens_mtw_manager`
- `ros2 run xsens_mtw_driver_ros2 xsens_mtw_visualization`
- `ros2 launch xsens_mtw_driver_ros2 xsens_mtw_visualization.launch.py` (uses `config/imu_mapping.yaml`) [experimental]


The driver upsamples the IMU data and publishes all sensor data into the `/xsens_imu_data` topic. \
Custom messages `IMUData.msg`, `IMUDataArray.msg` and `Quaternion.msg` are used. \
The `params.yaml` is currently not used, due to the [problem](#problems) listed below. \
The `imu_mapping.yaml` is only used for a specific IMU setup. It will just move the orientations in a more "visually correct" position. Using `xsens_mtw_visualization` without any config, will just publish the TFs of all IMUs next to each other for visibility.

Supported sensor update rates for the Xsens MTw Awinda System:

|    IMUs  | desiredUpdateRate (max) |
|----------|-------------------------|
|   1 - 5  |           120 Hz        |
|   6 - 9  |           100 Hz        |
|      10  |            80 Hz        |
| 11 - 20  |            60 Hz        |
| 21 - 32  |            40 Hz        |

## Problems

- Starting the `xsens_mtw_manager` node through launch files will prevent keyinput readings from `conio.c`, meaning the `params.yaml` would need to be loaded set up by a separate parameter server node.
- CTRL+C does not execute the node destructor, thus not freeing the master device

## Xsens MTw Awinda IMU Orientation

<img src="xsens_mtw_awinda_imu.png" alt="Image Description" width="200" height="180">


## Troubleshooting

Make sure you are in the correct group:

```
$ ls -l /dev/ttyUSB0

crw-rw---- 1 root dialout 188, 0 May 4 13:37 /dev/ttyUSB0

$ groups

"username" adm cdrom sudo dip plugdev lpadmin sambashare
```

Add yourself to the group: 
```
$ sudo usermod -G dialout -a $USER
$ newgrp dialout
```

More troubleshooting on the [xsens_mti_driver page](http://wiki.ros.org/xsens_mti_driver)

### VSCode ROS2 Coding:

Make sure to add `"/opt/ros/\<ros2version\>/include/**"` to the `includePath` in your `c_cpp_properties.json` from your `.vscode` folder
<>

## Related links

- [Xsens Software & Documentation](https://www.movella.com/support/software-documentation)
- [VQF - A Versatile Quaternion-based Filter for IMU Orientation Estimation](https://vqf.readthedocs.io/)
- [qleonardolp - xsens_mtw_driver-release](https://github.com/qleonardolp/xsens_mtw_driver-release)
- [ChangcongWang - xsens_mtw_driver-release [fork] (ROS2 Migration with Xsens SDK 4.6)](https://github.com/ChangcongWang/xsens_mtw_driver-release)
- ...
- `TODO`