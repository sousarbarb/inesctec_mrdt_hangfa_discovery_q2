# LDROBOT LD-19

## Links

- [LDROBOT D300 LiDAR Kit](https://www.ldrobot.com/ProductDetails?sensor_name=D300+Kit)
  _(LiDAR LD-19 + serial cable + speed control plate)_

## Documents

- [LDROBOT LD-19 Datasheet](../../../assets/sensors/lidar2d/ldrobot_d300_ld19/ldrobot_d300_ld19_datasheet.pdf)
- [LDROBOT LD-19 Development Manual](../../../assets/sensors/lidar2d/ldrobot_d300_ld19/ldrobot_d300_ld19_dev-manual.pdf)
- [LDROBOT LD-19 3D Model (STEP)](../../../assets/sensors/lidar2d/ldrobot_d300_ld19/ldrobot_d300_ld19_3d-model.step)

## Setup

!!! Note

    The setup of specific udev rules for USB-based 2D laser scanners may not be
    possible because it is the _CP2102N USB to UART Bridge Controller_
    (or other alike) that is communicating with the operating system and not the
    sensor itself...

    ```sh
    udevadm info -a -n /dev/ttyUSB0
    ```

### ROS 1 Noetic

```sh
source /opt/ros/noetic/setup.bash

cd ~/ros1_ws/src/
git clone git@github.com:ldrobotSensorTeam/ldlidar_ros.git

cd ldlidar_ros/
git submodule update --init --recursive -f

cd ~/ros1_ws/
rosdep install --from-paths src --ignore-src  -r -y
catkin_make --force-cmake -DCMAKE_BUILD_TYPE=Release

source devel/setup.bash
```

### ROS 2 Foxy

```sh
source /opt/ros/foxy/setup.bash

cd ~/ros2_ws/src/
git clone git@github.com:ldrobotSensorTeam/ldlidar_ros2.git

cd ldlidar_ros2/
git submodule update --init --recursive -f

cd ~/ros2_ws/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers summary+ status+ console_cohesion+ console_direct+ console_start_end+ console_stderr+

source install/setup.bash
```

## Launch

### ROS 1 Noetic

```sh
source /opt/ros/noetic/setup.bash
source /home/user/ros1_ws/devel/setup.bash

roslaunch ldlidar_ros ld19.launch
```

```xml title="ld19.launch"
<?xml version="1.0"?>
<launch>
  <arg name="laser_scan_topic_name"     default="scan"/>
  <arg name="point_cloud_2d_topic_name" default="pointcloud2d"/>
  <arg name="frame_id"                  default="base_laser"/>
  <arg name="port_name"                 default="/dev/ttyUSB0"/>
  <arg name="fix_to_base_link"          default="true" />

  <!-- LDROBOT LiDAR message publisher node -->
  <node pkg="ldlidar_ros" type="ldlidar_ros_node"
        name="ldlidar_publisher_ld19"
        output="screen">

    <param name="product_name"              value="LDLiDAR_LD19"/>
    <param name="laser_scan_topic_name"     value="$(arg laser_scan_topic_name)"/>
    <param name="point_cloud_2d_topic_name" value="$(arg point_cloud_2d_topic_name)"/>
    <param name="frame_id"                  value="$(arg frame_id)"/>
    <param name="port_name"                 value="$(arg port_name)"/>
    <param name="serial_baudrate"           value="230400"/>

    <!-- Set laser scan directon: -->
    <!--    1. Set counterclockwise, example: <param name="laser_scan_dir" type="bool" value="true"/> -->
    <!--    2. Set clockwise,        example: <param name="laser_scan_dir" type="bool" value="false"/> -->
    <param name="laser_scan_dir" type="bool" value="true"/>

    <!-- Angle crop setting, Mask data within the set angle range -->
    <!--    1. Enable angle crop fuction: -->
    <!--       1.1. enable angle crop,  example: <param name="enable_angle_crop_func" type="bool" value="true"/> -->
    <!--       1.2. disable angle crop, example: <param name="enable_angle_crop_func" type="bool" value="false"/> -->
    <param name="enable_angle_crop_func" type="bool" value="false"/>

    <!--    2. Angle cropping interval setting, The distance and intensity data within the set angle range will be set to 0 -->
    <!--       angle >= "angle_crop_min" and angle <= "angle_crop_max", unit is degress -->
    <param name="angle_crop_min" type="double" value="135.0"/>
    <param name="angle_crop_max" type="double" value="225.0"/>
    <param name="range_min" type="double" value="0.02"/>
    <param name="range_max" type="double" value="12.0"/>

  </node>

  <!-- publisher tf transform, parents frame is base_link, child frame is base_laser -->
  <!-- args="x y z yaw pitch roll parents_frame_id child_frame_id period_in_ms"-->
  <node name="base_to_laser_ld19" pkg="tf" type="static_transform_publisher"
        args="0.0 0.0 0.18 0 0.0 0.0 base_link base_laser 50"
        if="$(arg fix_to_base_link)"/>

</launch>
```

### ROS 2 Foxy

```sh
source /opt/ros/foxy/setup.bash
source /home/user/ros2_ws/install/setup.bash

ros2 launch ldlidar_ros2 ld19.launch.py
```

```py title="ld19.launch.py"
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan directon:
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar_ros2',
      executable='ldlidar_ros2_node',
      name='ldlidar_publisher_ld19',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'laser_scan_topic_name': 'scan'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'serial_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},  # unit is degress
        {'angle_crop_max': 225.0},  # unit is degress
        {'range_min': 0.02}, # unit is meter
        {'range_max': 12.0}   # unit is meter
      ]
  )

  # base_link to base_laser tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ld19',
    arguments=['0','0','0.18','0','0','0','base_link','base_laser']
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)

  return ld
```
