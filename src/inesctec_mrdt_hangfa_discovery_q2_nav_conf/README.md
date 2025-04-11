# inesctec_mrdt_hangfa_discovery_q2_nav_conf

This repository implements the launch files required to operate the
[Hangfa Discovery Q2](http://www.hangfa.com/EN/robot/DiscoveryQ2.html)
four-wheeled omnidirectional robot, one of the "twin" yellow robots.
The system implemented is based on the INESC TEC Robotics Navigation Stack that
it allows you to have different configurations implemented and selecting just
one based on your environment variables.

**Version 0.3.0**

**With this version, it is possible to do:**

- Basic environment configuration (firmware driver, joystick, wheeled odometry)
- Basic 2D laser scanner configuration (basic + laser 2D) w/ Hokuyo UST-10LX
- Basic 3D LiDAR configuration (basic + lidar 3D) w/ Livox Mid-360

**The next version will add these features:**

- TBD

## Environment Configurations

- `basic`
  - drivers
    - inesctec_mrdt_hangfa_discovery_q2_driver
    - inesctec_mrdt_omnijoy_driver_logif710
  - hmi
    - rviz
  - localization
    - inesctec_mrdt_localization_odom
- `basic_2d`
  - drivers
    - inesctec_mrdt_hangfa_discovery_q2_driver
    - inesctec_mrdt_omnijoy_driver_logif710
    - static_transform_publisher (laser > base_footprint)
    - urg_node
  - hmi
    - rviz
  - localization
    - inesctec_mrdt_localization_odom
  - **Additional configurations:**
    - Network:
      1. Open the Settings > Network
      2. Select the _Hokuyo_ wired interface settings
- `basic_3d`
  - drivers
    - livox_ros_driver2
    - inesctec_mrdt_hangfa_discovery_q2_driver
    - inesctec_mrdt_omnijoy_driver_logif710
    - static_transform_publisher (lidar3d > base_footprint)
  - hmi
    - rviz
  - localization
    - inesctec_mrdt_localization_odom
  - **Additional configurations:**
    - Network:
      1. Open the Settings > Network
      2. Select the _Livox Mid-360_ / _RoboSense Helios_ wired interface
         settings

## ROS

**ROS 1**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

**ROS 2**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS 2 Foxy](https://docs.ros.org/en/foxy/)

## Usage

### Setup

```sh
# Robot id
export ROBOT_ID=<id>                # (default: unnamed_robot)
# Configuration
export ROBOT_CONF=<configuration>   # (default: basic)
```

### Compilation

**ROS 1**

```sh
# ROS 1 environment setup
source /opt/ros/noetic/setup.bash

# Clone the repository
git clone git@github.com:sousarbarb/inesctec_mrdt_hangfa_discovery_q2.git

# ROS 1 workspace setup
cd inesctec_mrdt_hangfa_discovery_q2/src
catkin_init_workspace

# Build
cd ..
catkin_make --force-cmake -DCMAKE_BUILD_TYPE=Release
# OR catkin_make_isolated (more slow, build and check dependencies individually)
# OR catkin build (requires the Pyhton-based catkin tools)
source devel/setup.bash
```

**ROS 2**

```sh
# ROS 2 environment setup
source /opt/ros/foxy/setup.bash

# Clone the repository
git clone git@github.com:sousarbarb/inesctec_mrdt_hangfa_discovery_q2.git

# Build
cd inesctec_mrdt_hangfa_discovery_q2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers summary+ status+ console_cohesion+ console_direct+ console_start_end+ console_stderr+
source install/setup.bash
```

### Launch

**ROS 1**

```sh
roslaunch inesctec_mrdt_hangfa_discovery_q2_nav_conf wake_up_dumb_q2.launch
```

**ROS 2**

```sh
ros2 launch inesctec_mrdt_hangfa_discovery_q2_nav_conf wake_up_dumb_q2.launch.xml
```

## Acknowledgements

- [5dpo Robotics Team](https://5dpo.github.io/)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)
