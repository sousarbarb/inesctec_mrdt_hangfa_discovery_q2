# Hokuyo UST-10LX

## Links

- [Hokuyo UST-10LX](https://www.hokuyo-aut.jp/search/single.php?serial=167)

## Documents

- [Hokuyo UST-10LX Instruction Manual](../../../assets/sensors/lidar2d/hokuyo_ust-10lx/hokuyo_ust-10lx_instruction-manual.pdf)
- [Hokuyo UST-10LX Datasheet](../../../assets/sensors/lidar2d/hokuyo_ust-10lx/hokuyo_ust-10lx_datasheet.pdf)
- [Hokuyo UST-10LX-H01 Datasheet](../../../assets/sensors/lidar2d/hokuyo_ust-10lx/hokuyo_ust-10lx-h01_datasheet.pdf)
- [Hokuyo UST-10LX-H02 Datasheet](../../../assets/sensors/lidar2d/hokuyo_ust-10lx/hokuyo_ust-10lx-h02_datasheet.pdf)
- [Hokuyo UST-10LX UST Communication Protocol Specification](../../../assets/sensors/lidar2d/hokuyo_ust-10lx/hokuyo_ust-10lx_ust-communication-protocol.pdf)
- [Hokuyo UST-10LX IP Discovery Tool](../../../assets/sensors/lidar2d/hokuyo_ust-10lx/hokuyo_ust-10lx_ip-discovery-tool_v2.1.zip)
- [Hokuyo UST-10LX 3D Model (STEP)](../../../assets/sensors/lidar2d/hokuyo_ust-10lx/hokuyo_ust-10lx_3d-model.STEP)

!!! Note

    Indeed, there are two versions of the Hokuyo UST-10LX sensor: H01 (most
    typical one, indoor usage) and H02 (outdoor environments). As a result, the
    latter supports a much higher surrounding intensity specification than the
    former (< 80.000lx versus < 15.000lx for H02 and H01, respectively).

## Setup

### Network

1. Open Ubuntu Settings
    - Task Bar :material-triangle-down: :material-arrow-right:
      :material-wrench: Settings
    - Dock :material-arrow-right: :material-dots-grid: Show Applications
      :material-arrow-right: :material-wrench: Settings
2. :material-earth: Network :material-arrow-right: Wired :material-arrow-right:
   :material-wrench: Settings :material-arrow-right: IPv4
    - Set _IPv4 Method_ to _Manual_
    - Addresses
        - Address: `192.168.0.1`
        - Netmask: `255.255.255.0`

!!! Info

    The default network setting of the Hokuyo UST-10LX is the following one:

    - IP address: `192.168.0.10`
    - Port number: `10940`

    As a result, the computer should be in the same IP range, following a
    `255.255.255.0` mask for the IP address.

!!! Info

    If you want to change the IP address of the Hokuyo sensor, use the
    [IP Discovery Tool](../../../assets/sensors/lidar2d/hokuyo_ust-10lx/hokuyo_ust-10lx_ip-discovery-tool_v2.1.zip).

!!! Tip

    One way to discover the IP address of the Hokuyo UST-10LX sensor is
    (assuming that the Ethernet wired connection is already configured to
    _Manual IPv4 Method_ and its static IP is in the same range of the sensor):

    ```sh
    nmap 192.168.0.0/24 -sn       # ping scan (disables port scan, should be faster)
    nmap 192.168.0.0/24 -p 10940  # only scan a specific port
    nmap 192.168.0.0/24 -p1-65535 # scans a range of ports (1 to 65535)
    ```

### ROS 1 Noetic

```sh
sudo apt install ros-noetic-urg-node
```

### ROS 2 Foxy

```sh
sudo apt install ros-foxy-urg-node
```

## Launch

### ROS 1 Noetic

```sh
source /opt/ros/noetic/setup.bash

mkdir ~/ros1_ws/launch/urg_node/ -p
cd ~/ros1_ws/launch/urg_node
touch hokuyo_ust10lx.launch
tee hokuyo_ust10lx.launch <<EOF
<launch>

  <node pkg="urg_node" type="urg_node" name="urg_node" output="screen">

    <param name="ip_address"        value="192.168.0.10"/>
    <param name="ip_port"           value="10940"/>
    <param name="serial_port"       value=""/>
    <param name="serial_baud"       value=""/>
    <param name="frame_id"          value="laser"/>
    <param name="calibrate_time"    value="false"/>
    <param name="publish_intensity" value="false"/>
    <param name="publish_multiecho" value="false"/>
    <param name="angle_min"         value="-2.2689"/>
    <param name="angle_max"         value="2.2689"/>

  </node>

</launch>

EOF

roslaunch hokuyo_ust10lx.launch
```

```xml title="hokuyo_ust10lx.launch"
<launch>

  <node pkg="urg_node" type="urg_node" name="urg_node" output="screen">

    <param name="ip_address"        value="192.168.0.10"/>
    <param name="ip_port"           value="10940"/>
    <param name="serial_port"       value=""/>
    <param name="serial_baud"       value=""/>
    <param name="frame_id"          value="laser"/>
    <param name="calibrate_time"    value="false"/>
    <param name="publish_intensity" value="false"/>
    <param name="publish_multiecho" value="false"/>
    <param name="angle_min"         value="-2.2689"/>
    <param name="angle_max"         value="2.2689"/>

  </node>

</launch>
```

### ROS 2 Foxy

```sh
source /opt/ros/foxy/setup.bash

mkdir ~/ros2_ws/launch/urg_node/ -p
cd ~/ros2_ws/launch/urg_node
touch hokuyo_ust10lx.launch.xml
tee hokuyo_ust10lx.launch.xml <<EOF
<launch>

  <node pkg="urg_node" exec="urg_node_driver" name="urg_node">

    <param name="ip_address"        value="192.168.0.10"/>
    <param name="ip_port"           value="10940"/>
    <param name="laser_frame_id"    value="laser"/>
    <param name="calibrate_time"    value="false"/>
    <param name="publish_intensity" value="false"/>
    <param name="publish_multiecho" value="false"/>
    <param name="angle_min"         value="-2.2689"/>
    <param name="angle_max"         value="2.2689"/>

  </node>

</launch>

EOF

ros2 launch hokuyo_ust10lx.launch.xml
```

```xml title="hokuyo_ust10lx.launch.xml"
<launch>

  <node pkg="urg_node" exec="urg_node_driver" name="urg_node">

    <param name="ip_address"        value="192.168.0.10"/>
    <param name="ip_port"           value="10940"/>
    <param name="laser_frame_id"    value="laser"/>
    <param name="calibrate_time"    value="false"/>
    <param name="publish_intensity" value="false"/>
    <param name="publish_multiecho" value="false"/>
    <param name="angle_min"         value="-2.2689"/>
    <param name="angle_max"         value="2.2689"/>

  </node>

</launch>
```
