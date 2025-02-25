# ROS 1 Noetic Ninjemys

## Links

- [ROS Noetic Ninjemys](https://wiki.ros.org/noetic) _(EOL: May, 2025)_
- [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials)
- [Unix Tutorial for Beginners](https://info-ee.surrey.ac.uk/Teaching/Unix/)
- [ROS Courses](https://wiki.ros.org/Courses)

## Installation

```sh
sudo apt update
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y
source /opt/ros/noetic/setup.bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo rosdep init
rosdep update
```

See [https://wiki.ros.org/noetic/Installation/Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu)
for further details on the ROS installation instructions.

In order to test if the installation was successful, run the following code:

```sh
source /opt/ros/noetic/setup.bash
roslaunch roscpp_tutorials talker_listener.launch
```

## Configuration of the ROS Environment

```sh
source /opt/ros/noetic/setup.bash

mkdir ~/ros1_ws/src -p

cd ~/ros1_ws/src/
catkin_init_workspace .

cd ..
catkin_make --force-cmake -DCMAKE_BUILD_TYPE=<Debug|Release|RelWithDebInfo|MinSizeRel>

source devel/setup.bash

cat <<EOF >> ~/.bashrc

## ROBOT OPERATING SYSTEM (ROS) 1 NOETIC
wsros1dir()
{
  source devel/setup.bash
}
alias wsros1='source /opt/ros/noetic/setup.bash && source ${HOME}/ros1_ws/devel/setup.bash'
EOF
```

## Network Setup

!!! Tip

    Use [ZeroTier](https://www.zerotier.com/) (see [Remote Access](remote.md))
    to create a private network link between your computer and the robots'
    computing units.

**Requirements**

- IP address of your computer
- IP address of the robot's computing unit

!!! Tip

    Ideally, both IP addresses should remain static
    (another advantage of using [ZeroTier](https://www.zerotier.com/) for that
    effect).

**How To**

1. Add your IP address and hostname to the `/etc/hosts` file in robot's
   computing unit
    ```sh
    user@discoveryq2:~$ cat /etc/hosts
    127.0.0.1           localhost
    127.0.1.1           <ROBOT HOSTNAME>
    <YOUR IP ADDRESS>   <YOUR HOSTNAME>
    ...
    ```
2. Add the robot's computing unit IP address and hostname to the `/etc/hosts`
   file in your computer
    ```sh
    <YOUR USERNAME>@<YOUR HOSTNAME>:~$ cat /etc/hosts
    127.0.0.1           localhost
    127.0.1.1           <YOUR HOSTNAME>
    <ROBOT ADDRESS>     <ROBOT HOSTNAME>
    ...
    ```
3. Test if the naming resolve setup is working correctly
    ```sh
    user@discoveryq2:~$ ping <YOUR HOSTNAME>
    ```
    ```sh
    <YOUR USERNAME>@<YOUR HOSTNAME>:~$ ping <ROBOT HOSTNAME>
    ```
4. Launch nodes in the robot's local computing unit
5. Execute the following commands in your computer
    ```sh
    export ROS_MASTER_URI=http://<ROBOT HOSTNAME>:11311
    rosnode list
    ```
6. Now, you can remotely execute additional nodes such as _rviz_ to see data
   published by nodes running on the robot, debug, among other use cases

More information available in
[https://wiki.ros.org/ROS/NetworkSetup](https://wiki.ros.org/ROS/NetworkSetup).
