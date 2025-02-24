# Single Board Computer (SBC) Unit

The Single Board Computer (SBC) serves as the central computing unit of the
modified Hangfa Discovery Q2 platform. This computing unit is responsible for
communicating with the microcontroller via a serial port connection, executing
[Robot Operating System (ROS)](https://ros.org/)-based nodes, and interfacing
with external systems.

While Raspberry Pi SBCs could have been a more cost-effective alternative to
x86-based boards, they often face compatibility issues with certain libraries
due to their ARM architecture - a limitation shared with Jetson boards.
Additionally, Raspberry Pi boards may lack the necessary processing power for
multimodal perception applications, even when only recording data.

As a result, we opted for LattePanda SBCs, specifically, the
[LattePanda 3 Delta](https://www.lattepanda.com/lattepanda-3-delta) (used in our
experiments) and
[LattePanda Sigma](https://www.lattepanda.com/lattepanda-sigma)
(a more powerful alternative) boards. This guide provides detailed, step-by-step
instructions for intermediate and advance  on how to:

- Set up the SBCs (BIOS settings, power, storage, and display configuration)
- Install the operating system
  ([Ubuntu Focal Fossa 20.04 LTS](https://releases.ubuntu.com/focal/))
- Install and configure the ROS framework
  ([ROS 1 Noetic Ninjemys](https://wiki.ros.org/noetic) and
  [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/))
- Configure remote desktop access
- Set up the development environment
- Firmware setup in terms of udev rules for peripheral management, and
  installation and configuration of the advanced serial monitor application
  [CoolTerm](https://freeware.the-meiers.org/)

Following these step-by-step instructions will ensure that you are capable of
replicating our configuration in terms of computing unit, while may also give
you additional ideas on how to improve it even further.