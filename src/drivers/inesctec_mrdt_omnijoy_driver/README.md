# inesctec_mrdt_omnijoy_driver

**Version 0.1.0**

This repository implements a ROS driver to control an omnidirectional robot
(but also compatible with other steering geometries) with a joystick controller.

**With this version, it is possible to do:**

- Joystick control of a mobile robot
- Dynamic reconfigure of the node parameters

**The next version will add these features:**

No further development is foreseen for this package.

## ROS

**ROS 1**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

**ROS 2**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS 2 Foxy](https://docs.ros.org/en/foxy/)

### Dependencies

- [rclcpp](https://index.ros.org/r/rclcpp/) (_ROS 2_)
- [roscpp](https://wiki.ros.org/roscpp/) (_ROS 1_)
- [dynamic_reconfigure](https://index.ros.org/p/dynamic_reconfigure/) (_ROS 1_)
- [geometry_msgs](https://index.ros.org/p/geometry_msgs/)
- [joy](https://index.ros.org/p/joy/)
- [sensor_msgs](https://index.ros.org/p/sensor_msgs/)

### Parameters

- axis_linear_x (`int = 1`): index number of x linear axis (`/joy.axes`)
- axis_linear_y (`int = 2`): index number of y linear axis (`/joy.axes`)
- axis_angular (`int = 0`): index number of angular axis (`/joy.axes`)
- axis_deadman (`int = 4`): index number of button for deadman switch
  (`/joy.buttons`)
- axis_turbo (`int = 5`): index number of button for turbo (`/joy.buttons`)
- axis_turbo_up (`int = 6`): index number of button for increasing boost
  (`/joy.buttons`)
- axis_turbo_down (`int = 7`): index number of button for decreasing boost
  (`/joy.buttons`)
- scale_linear (`double = 0.1`): linear scale to normalize (m/s)
- scale_angular (`double = 0.2`): angular scale to normalize (rad/s)
- turbo_scale_linear (`double = 0.2`): use this scale instead of `scale_linear`
  when pressing the turbo button (m/s)
- turbo_max_scale_linear (`double = 0.4`): maximum linear turbo scale (m/s)
- turbo_scale_angular (`double = 0.4`): use this scale instead of
  `scale_angular` when pressing the turbo button (rad/s)
- turbo_max_scale_angular (`double = 0.8`): maximum angular turbo scale (rad/s)

In order to know the index numbers of your controller, go to
[joy](https://wiki.ros.org/joy) documentation, _5. Application_, to see the
options available for those parameters.

### Subscribes

- joy (`sensor_msgs::Joy.msg`)

### Publishes

- cmd_vel (`geometry_msgs::Twist.msg`)

### Services

None.

### Actions

None.

## Usage

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
roslaunch inesctec_mrdt_omnijoy_driver inesctec_mrdt_omnijoy_driver_logif710.launch
```

**ROS 2**

```sh
ros2 launch inesctec_mrdt_omnijoy_driver inesctec_mrdt_omnijoy_driver_logif710.launch.xml
```

## Acknowledgements

- [@fedetft](https://github.com/fedetft)
- [5dpo Robotics Team](https://5dpo.github.io/)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)
