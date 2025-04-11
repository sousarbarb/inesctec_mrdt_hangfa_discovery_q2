#!/bin/bash

sleep 5

source $HOME/adam_ros1_ws/devel/setup.bash

gnome-terminal -- roscore

export command2execute='roslaunch adam_nav_conf wake_up_dumb_adam.launch --wait'

gnome-terminal -- ${command2execute}
