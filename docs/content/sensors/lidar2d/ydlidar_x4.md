# YDLIDAR X4

## Links

- [YDLIDAR X4](https://www.ydlidar.com/products/view/5.html)

## Documents

- [YDLIDAR X4 Datasheet](../../../assets/sensors/lidar2d/ydlidar_x4/ydlidar_x4_datasheet.pdf)
- [YDLIDAR X4 User Manual](../../../assets/sensors/lidar2d/ydlidar_x4/ydlidar_x4_user-manual.pdf)
- [YDLIDAR X4 3D Model (STEP)](../../../assets/sensors/lidar2d/ydlidar_x4/ydlidar_x4_3d-model.step)

## Setup

!!! Note

    The setup of specific udev rules for USB-based 2D laser scanners may not be
    possible because it is the _CP2102N USB to UART Bridge Controller_
    (or other alike) that is communicating with the operating system and not the
    sensor itself...

    ```sh
    udevadm info -a -n /dev/ttyUSB0
    ```

### YDLIDAR SDK

```sh
# Dependencies
sudo apt install cmake pkg-config

mkdir ~/dev/ros -p

cd ~/dev/ros
git clone git@github.com:YDLIDAR/YDLidar-SDK.git

cd YDLidar-SDK
mkdir build

cd build
cmake ..
make
sudo make install
```

### ROS 1 Noetic

```sh
cd ~/ros1_ws/src/
git clone git@github.com:YDLIDAR/ydlidar_ros_driver.git

cd ~/ros1_ws/
catkin_make --force-cmake -DCMAKE_BUILD_TYPE=Release

source devel/setup.bash
```

### ROS 2 Foxy

```sh
cd ~/ros2_ws/src/
git clone git@github.com:YDLIDAR/ydlidar_ros2_driver.git

cd ~/ros2_ws/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers summary+ status+ console_cohesion+ console_direct+ console_start_end+ console_stderr+

source install/setup.bash
```

## Launch

### ROS 1 Noetic

```sh
source /opt/ros/noetic/setup.bash
source /home/user/ros1_ws/devel/setup.bash

roslaunch ydlidar_ros_driver X4.launch
```

```xml title="X4.launch"
<launch>

  <node pkg="ydlidar_ros_driver" type="ydlidar_ros_driver_node"
        name="ydlidar_lidar_publisher"
        output="screen" respawn="false">

    <!-- string property -->
    <param name="port"         type="string" value="/dev/ttyUSB0"/>
    <param name="frame_id"     type="string" value="laser_frame"/>
    <param name="ignore_array" type="string" value=""/>

    <!-- int property -->
    <param name="baudrate"             type="int" value="128000"/>
    <!-- 0:TYPE_TOF, 1:TYPE_TRIANGLE, 2:TYPE_TOF_NET -->
    <param name="lidar_type"           type="int" value="1"/>
    <!-- 0:YDLIDAR_TYPE_SERIAL, 1:YDLIDAR_TYPE_TCP -->
    <param name="device_type"          type="int" value="0"/>
    <param name="sample_rate"          type="int" value="5"/>
    <param name="abnormal_check_count" type="int" value="4"/>

    <!-- bool property -->
    <param name="resolution_fixed"         type="bool" value="true"/>
    <param name="auto_reconnect"           type="bool" value="true"/>
    <param name="reversion"                type="bool" value="false"/>
    <param name="inverted"                 type="bool" value="true"/>
    <param name="isSingleChannel"          type="bool" value="false"/>
    <param name="intensity"                type="bool" value="false"/>
    <param name="support_motor_dtr"        type="bool" value="true"/>
    <param name="invalid_range_is_inf"     type="bool" value="false"/>
    <param name="point_cloud_preservative" type="bool" value="false"/>

    <!-- float property -->
    <param name="angle_min" type="double" value="-180"/>
    <param name="angle_max" type="double" value="180"/>
    <param name="range_min" type="double" value="0.1"/>
    <param name="range_max" type="double" value="12.0"/>
    <!-- frequency is invalid, External PWM control speed -->
    <param name="frequency" type="double" value="10.0"/>

  </node>

</launch>
```

!!! Tip

    Default launch file uses `/dev/ydlidar` port name, you probably need to
    change it to `/dev/ttyUSB0`.

### ROS 2 Foxy

```sh
source /opt/ros/foxy/setup.bash
source /home/user/ros2_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:=/home/user/ros2_ws/src/ydlidar_ros2_driver/params/X4.yaml
```

```py title="ydlidar_launch.py"
#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ydlidar_ros2_driver_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'ydlidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='ydlidar_ros2_driver',
                                node_executable='ydlidar_ros2_driver_node',
                                node_name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                node_namespace='/',
                                )
    tf2_node = Node(package='tf2_ros',
                    node_executable='static_transform_publisher',
                    node_name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
    ])
```

```yaml title="X4.yaml"
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    ignore_array: ""
    baudrate: 128000
    lidar_type: 1
    device_type: 0
    sample_rate: 5
    abnormal_check_count: 4
    fixed_resolution: true
    reversion: true
    inverted: true
    auto_reconnect: true
    isSingleChannel: false
    intensity: false
    support_motor_dtr: true
    angle_max: 180.0
    angle_min: -180.0
    range_max: 12.0
    range_min: 0.1
    frequency: 10.0
    invalid_range_is_inf: false
```
