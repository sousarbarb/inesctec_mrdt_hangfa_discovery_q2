# [adam_nav_conf](https://gitlab.inesctec.pt/mrdt/mobile-robots/adam/adam_nav_conf)

This repository implements the launch files required to operate the
**Adam** [Hangfa Discovery Q2](http://www.hangfa.com/EN/robot/DiscoveryQ2.html)
four-wheeled omnidirectional robot
([GitLab](https://gitlab.inesctec.pt/mrdt/mobile-robots/adam),
[Redmine](http://criis-projects.inesctec.pt/projects/robot-adam)),
one of the "twin" yellow robots.
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
    - sdpo_hangfaq2_driver
    - sdpo_omnijoy_driver_logif710
  - hmi
    - rviz
  - localization
    - sdpo_localization_odom
- `basic_2d`
  - drivers
    - sdpo_hangfaq2_driver
    - sdpo_omnijoy_driver_logif710
    - static_transform_publisher (laser > base_footprint)
    - urg_node
  - hmi
    - rviz
  - localization
    - sdpo_localization_odom
  - **Additional configurations:**
    - Network:
      1. Open the Settings > Network
      2. Select the _Hokuyo_ wired interface settings
- `basic_3d`
  - drivers
    - livox_ros_driver2
    - sdpo_hangfaq2_driver
    - sdpo_omnijoy_driver_logif710
    - static_transform_publisher (lidar3d > base_footprint)
  - hmi
    - rviz
  - localization
    - sdpo_localization_odom
  - **Additional configurations:**
    - Network:
      1. Open the Settings > Network
      2. Select the _Livox Mid-360_ wired interface settings

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
source source /opt/ros/noetic/setup.bash

# Create workspace
mkdir -p ~/ros1_ws/src

# Clone the repository
cd ~/ros1_ws/src
git clone git@gitlab.inesctec.pt:mrdt/mobile-robots/adam/adam_nav_conf.git

# Build
cd ~/ros1_ws
catkin_make
# OR catkin_make_isolated (more slow, build and check dependencies individually)
# OR catkin build (requires the Pyhton-based catkin tools)
source devel/setup.bash
```

**ROS 2**

```sh
# ROS 2 environment setup
source /opt/ros/foxy/setup.bash

# Create workspace
mkdir -p ~/ros2_ws/src

# Clone the repository
cd ~/ros2_ws/src
git clone git@gitlab.inesctec.pt:mrdt/mobile-robots/adam/adam_nav_conf.git

# Build
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Launch

**ROS 1**

```sh
roslaunch adam_nav_conf wake_up_dumb_adam.launch
```

**ROS 2**

```sh
ros2 launch adam_nav_conf wake_up_dumb_adam.launch.xml
```

## Acknowledges

- [5dpo Robotics Team](https://5dpo.github.io/)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Héber Miguel Sobreira ([github](https://github.com/HeberSobreira),
  [gitlab](https://gitlab.inesctec.pt/heber.m.sobreira),
  [mail](mailto:heber.m.sobreira@inesctec.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.inesctec.pt/ricardo.b.sousa),
  [mail:inesctec](mailto:ricardo.b.sousa@inesctec.pt),
  [mail:personal](mailto:sousa.ricardob@outlook.com),
  [mail:professor](mailto:rbs@fe.up.pt),
  [mail:student](mailto:up201503004@edu.fe.up.pt))
