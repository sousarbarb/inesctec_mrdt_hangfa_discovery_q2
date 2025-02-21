# ROS 2 Foxy Fitzroy

## Links

- [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/) _(EOL: June, 2023)_
- [ROS 2 Foxy Official Tutorials](https://docs.ros.org/en/foxy/Tutorials.html)
- [Unix Tutorial for Beginners](https://info-ee.surrey.ac.uk/Teaching/Unix/)
- [Programming for Robotics - ROS - RSL@ETHZ](https://rsl.ethz.ch/education-students/lectures/ros.html)

## Installation

```sh
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-foxy-desktop python3-argcomplete ros-dev-tools -y
```

See [https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
for further details on the ROS 2 Foxy installation instructions.

In order to test if the installation was successful, run the following code:

```sh
source /opt/ros/foxy/setup.bash
ros2 launch demo_nodes_cpp talker_listener.launch.xml
```

## Configuration of the ROS Environment

```sh
source /opt/ros/foxy/setup.bash

mkdir ~/ros2_ws/src -p

cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=<Debug|Release|RelWithDebInfo|MinSizeRel> --event-handlers summary+ status+ console_cohesion+ console_direct+ console_start_end+ console_stderr+

source install/setup.bash

cat <<EOF >> ~/.bashrc

## ROBOT OPERATING SYSTEM (ROS) 2 FOXY
wsros2dir()
{
  source install/setup.bash
}
alias wsros2='source /opt/ros/foxy/setup.bash && source ${HOME}/ros2_ws/install/setup.bash'
EOF
```
