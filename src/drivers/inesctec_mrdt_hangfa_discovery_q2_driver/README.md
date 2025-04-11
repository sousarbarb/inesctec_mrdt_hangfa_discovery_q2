# inesctec_mrdt_hangfa_discovery_q2_driver

**Version 2.1.1**

This repository implements a driver within a ROS package to communicate with the
firmware present in the Hangfa Q2 Discovery robot. The driver is required
for communicating with the robot and have available all its different functions.

The serial communication is handled by
[Boost.Asio](https://www.boost.org/doc/libs/1_80_0/doc/html/boost_asio.html).
This communication is based on the example `4_callback` provided in the
[serial-port](https://github.com/fedetft/serial-port) GitHub repository.

**With this version, it is possible to do:**

- Communicate with Arduino Mega 2560 using Boost.Asio
  (inesctec_mrdt_serial_port)
- Subscribe motors angular speed reference
- Publish encoders data (encoders + wheels angular speed)
- Read encoders
- Set motors speed
- Reset driver upon reset signal
- Watchdog timer to monitor the motors angular speed reference
- Send serial message to the firmware upon reconnection of the serial port
  communication
- Check automatically (1Hz) the status of the serial port communication

**The next version will add these features:**

No further development is expected for this package.

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
- inesctec_mrdt_drivers_interfaces
- inesctec_mrdt_serial_port
- inesctec_mrdt_serial_channels

### Parameters

- encoder_res (`float = 48.0`): resolution of the encoder (ticks/rot)
- gear_reduction (`float = 64.0`): reduction ratio of the transmissions
  (\[gear_reduction:1\])
- serial_port_name (`std::string = "/dev/ttyACM0"`): name of the serial port

### Subscribes

- motors_ref (`inesctec_mrdt_drivers_interfaces::MotRefArray.msg`)

### Publishes

- motors_enc (`inesctec_mrdt_drivers_interfaces::MotEncArray.msg`)

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
roslaunch inesctec_mrdt_hangfa_discovery_q2_driver inesctec_mrdt_hangfa_discovery_q2_driver.launch
```

**ROS 2**

```sh
ros2 launch inesctec_mrdt_hangfa_discovery_q2_driver inesctec_mrdt_hangfa_discovery_q2_driver.launch.xml
```

## Acknowledgements

- [5dpo Robotics Team](https://5dpo.github.io/)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)
