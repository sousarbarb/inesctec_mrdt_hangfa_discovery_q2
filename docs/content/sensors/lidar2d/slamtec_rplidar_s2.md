# SLAMTEC RPLIDAR S2

## Links

- [RPLIDAR S2](https://www.slamtec.com/en/S2)

## Documents

- [RPLIDAR S2 Datasheet](../../../assets/sensors/lidar2d/slamtec_rplidar_s2/slamtec_rplidar_s2_datasheet.pdf)
- [RPLIDAR S2 Development Kit User Manual](../../../assets/sensors/lidar2d/slamtec_rplidar_s2/slamtec_rplidar_s2_user-manual.pdf)
- [RPLIDAR S2 Interface Protocol](../../../assets/sensors/lidar2d/slamtec_rplidar_s2/slamtec_rplidar_s2_dev-manual.pdf)
- [RPLIDAR S2 3D Model (IGS)](../../../assets/sensors/lidar2d/slamtec_rplidar_s2/slamtec_rplidar_s2_3d-model.igs)

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
sudo apt install ros-noetic-rplidar-ros
```

### ROS 2 Foxy

```sh
# sudo apt install ros-foxy-rplidar-ros
# (much older version 2.0.2-1focal.20230527.062031
# compared to ros-noetic-rplidar-ros: 2.1.5-1focal.20240913.193218)

cd ~/ros2_ws/src/
git clone git@github.com:Slamtec/rplidar_ros.git

cd rplidar_ros/
git checkout ros2

cd ~/ros2_ws/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers summary+ status+ console_cohesion+ console_direct+ console_start_end+ console_stderr+

source install/setup.bash
```

## Launch

### ROS 1 Noetic

```sh
source /opt/ros/noetic/setup.bash

roslaunch rplidar_ros rplidar_s2.launch
```

```xml title="rplidar_s2.launch"
<launch>

  <node name="rplidarNode" pkg="rplidar_ros" type="rplidarNode"
        output="screen">

    <param name="serial_port"      type="string" value="/dev/ttyUSB0"/>
    <param name="serial_baudrate"  type="int"    value="1000000"/>
    <param name="frame_id"         type="string" value="laser"/>
    <param name="inverted"         type="bool"   value="false"/>
    <param name="angle_compensate" type="bool"   value="true"/>
    <param name="scan_frequency"   type="double" value="10.0"/>

  </node>

</launch>
```

### ROS 2 Foxy


```sh
source /opt/ros/foxy/setup.bash
source /home/user/ros2_ws/install/setup.bash

ros2 launch rplidar_ros rplidar_s2_launch.py
```

```py title="rplidar_s2_launch.py"
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type     = LaunchConfiguration('channel_type'    , default='serial')
    serial_port      = LaunchConfiguration('serial_port'     , default='/dev/ttyUSB0')
    serial_baudrate  = LaunchConfiguration('serial_baudrate' , default='1000000') #for s2 is 1000000
    frame_id         = LaunchConfiguration('frame_id'        , default='laser')
    inverted         = LaunchConfiguration('inverted'        , default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode        = LaunchConfiguration('scan_mode'       , default='DenseBoost')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode}],
            output='screen'),
    ])
```
