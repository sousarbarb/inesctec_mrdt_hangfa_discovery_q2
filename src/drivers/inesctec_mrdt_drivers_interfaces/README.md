# inesctec_mrdt_drivers_interfaces

This repository contains the ROS interfaces (actions, messages, services)
required to communicate with the robots' drivers / firmware.

## Robot Operating System (ROS)

**ROS 1**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

**ROS 2**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS 2 Foxy](https://docs.ros.org/en/foxy/)

### Dependencies

**ROS 1**

- [message_generation](https://wiki.ros.org/message_generation)
- [message_runtime](https://wiki.ros.org/message_runtime) (_runtime_)

**ROS 2**

- [builtin_interfaces](https://index.ros.org/p/builtin_interfaces/)
- [rosidl_default_generators](https://index.ros.org/p/rosidl_default_generators/)
- [rosidl_default_runtime](https://index.ros.org/p/rosidl_default_runtime/)
  (_runtime_)

## Usage

### Compilation

**ROS 1**

```sh
# ROS 1 environment setup
source source /opt/ros/noetic/setup.bash

# Create workspace
mkdir -p ~/ros1_ws/src

# Clone the repository
cd ~/ros1_ws/src
git clone git@github.com:sousarbarb/inesctec_mrdt_hangfa_discovery_q2.git

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
git clone git@github.com:sousarbarb/inesctec_mrdt_hangfa_discovery_q2.git

# Build
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Acknowledgements

- [5dpo Robotics Team](https://5dpo.github.io/)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)
