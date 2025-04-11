# [RoboSense RS-Helios 5515](https://www.robosense.ai/en/rslidar/RS-Helios)

## Specifications

| Description | Specification |
| :--- | :---: |
| #beams | 32 |
| Blind spot | 0.2m |
| Frame rate | 10 / 20Hz |
| Rotation speed | 600rpm @ 10Hz, 1200rpm @ 20Hz |
| HFOV | 360º |
| VFOV | 70º (-55º..+15º) |
| Horizontal resolution | 0.2º @ 10Hz, 0.4º @ 20Hz |
| Vertical resolution | up to 1.33º (not uniform) |
| Operating Voltage | 9..32V |
| Power consumption | 12W (typical), 20W (peak) |

## Setup

1. Power up the sensor
2. Connect an Ethernet cable to your PC
3. Configure a local network:
   - Manual
   - Address: 192.168.1.102
   - Netmask: 255.255.255.0
4. Validate the sensor configuration with the web browser: http://192.168.1.200/
   - **Return Mode:** due to beam divergence, multiple laser returns may appear
     from any single laser shot
     - Strongest
     - Last
     - First
     - Dual: details enhance, twice the volume of data versus single return mode
       - Sensor records both returns only when distance between two objects is
         greater than 1m
   - **Phase Lock:** when multiple lasers are used at the same time, with a PPS
     pulse signal; indicates the firing angle of the sensor's laser at the time
     of the rising edge of the PPS signal
   - **Reflectivity Enhance:** noise filter

## ROS

### Version

- ROS1
- ROS2

### Setup

**foxy**

1. Install a ROS 2 full desktop version
2. Download and install dependencies
   ```sh
   sudo apt-get install -y  libpcap-dev
   ```
3. Clone and configure the SDK repository
   (https://github.com/RoboSense-LiDAR/rslidar_sdk)
   ```sh
   # ROS 2
   source /opt/ros/foxy/setup.bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   
   git clone git@github.com:5dpo/rslidar_sdk.git
   git clone git@github.com:RoboSense-LiDAR/rslidar_msg.git

   cd rslidar_sdk
   git submodule init
   git submodule update --recursive

   cp package_ros2.xml package.xml
   nano CMakeLists.txt    # change CATKIN to COLCON

   # Build
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```
   - **Note:** a fork was made from the original SDK to support putting the
     config file in another folder

### Launch

```sh
roslaunch sdpo_hangfaq2_nav_conf run_rslidar_sdk.launch.xml
```
