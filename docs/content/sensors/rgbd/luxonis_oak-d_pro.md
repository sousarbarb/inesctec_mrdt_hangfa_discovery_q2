# Luxonis OAK-D Pro

## Links

- [OAK-D Pro Fixed-Focus](https://shop.luxonis.com/products/oak-d-pro?variant=42455252402399)
- [OAK-D Pro Documentation](https://docs.luxonis.com/hardware/products/OAK-D%20Pro)
- [DepthAI ROS](https://docs.luxonis.com/software/ros/depthai-ros/)

## Documents

- [OAK-D Pro Datasheet](../../../assets/sensors/rgbd/luxonis_oak-d_pro/luxonis_oak-d_pro_datasheet.pdf)
- [OAK-D Pro 3D Model (STEP)](../../../assets/sensors/rgbd/luxonis_oak-d_pro/luxonis_oak-d_pro_3d-model.stp)

## Setup

### DepthAI Dependencies

```sh
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

sudo wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | sudo bash
```

### ROS 1 Noetic

```sh
source /opt/ros/noetic/setup.bash

mkdir ~/dai_ros1_ws/src -p

cd ~/dai_ros1_ws/src
git clone --branch=noetic git@github.com:luxonis/depthai-ros.git

cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make_isolated

source devel_isolated/setup.bash
```

### ROS 2 Foxy

```sh
source /opt/ros/foxy/setup.bash

mkdir ~/dai_ros2_ws/src -p

cd ~/dai_ros2_ws/src/
git clone --branch=foxy git@github.com:luxonis/depthai-ros.git

cd ..
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-foxy-camera-info-manager ros-foxy-vision-msgs -y
MAKEFLAGS="-j1 -l1" colcon build

source install/setup.bash
```

## Launch

### ROS 1 Noetic

```sh
source /opt/ros/noetic/setup.bash
source /home/user/dai_ros1_ws/devel_isolated/setup.bash

roslaunch depthai_ros_driver camera.launch
```

```xml title="camera.launch"
<?xml version="1.0"?>
<launch>

	<arg name="publish_tf_from_calibration" default="false" />
	<arg name="imu_from_descr" default="false" />
	<arg name="override_cam_model" default="false" />
	<arg name="rectify_rgb" default="true" />
	<arg name="enable_pointcloud" default="false" />
	<arg name="enable_color" default="true" />
	<arg name="enable_depth" default="true" />
	<arg name="enable_infra1" default="false" />
	<arg name="enable_infra2" default="false" />
	<arg name="depth_module_depth_profile" default="1280,720,30" />
	<arg name="rgb_camera_color_profile" default="1280,720,30" />
	<arg name="depth_module_infra_profile" default="1280,720,30" />
	<arg name="rs_compat" default="false" />

	<arg name="name" default="camera" if="$(arg rs_compat)" />
	<arg name="name" default="oak" unless="$(arg rs_compat)" />

	<arg name="namespace" default="camera" if="$(arg rs_compat)" />
	<arg name="namespace" default="" unless="$(arg rs_compat)" />


	<arg name="parent_frame" value="$(arg name)_link" if="$(arg rs_compat)" />
	<arg name="parent_frame" default="oak-d-base-frame" unless="$(arg rs_compat)" />

	<arg name="points_topic_name" value="$(arg name)/depth/color/points" if="$(arg rs_compat)"/>
	<arg name="points_topic_name" value="$(arg name)/points" unless="$(arg rs_compat)"/>

	<arg name="color_sens_name" value="color" if="$(arg rs_compat)"/>
	<arg name="color_sens_name" value="rgb" unless="$(arg rs_compat)"/>

	<arg name="stereo_sens_name" value="depth" if="$(arg rs_compat)" />
	<arg name="stereo_sens_name" value="stereo" unless="$(arg rs_compat)" />

	<arg name="depth_topic_suffix" value="image_rect_raw" if="$(arg rs_compat)"/>
	<arg name="depth_topic_suffix" value="image_raw" unless="$(arg rs_compat)"/>



	<group if="$(arg rs_compat)">
		<param name="$(arg name)/camera_i_rs_compat" value="true" />
		<param name="$(arg name)/pipeline_gen_i_enable_sync" value="true" />
		<param name="$(arg name)/color_i_synced" value="true" />
		<param name="$(arg name)/color_i_publish_topic" value="$(arg enable_color)" />
		<param name="$(arg name)/color_i_width"
			value="$(eval rgb_camera_color_profile.split(',')[0])" />
		<param name="$(arg name)/color_i_height"
			value="$(eval rgb_camera_color_profile.split(',')[1])" />
		<param name="$(arg name)/color_i_fps" value="$(eval rgb_camera_color_profile.split(',')[2])" />

		<param name="$(arg name)/depth_i_synced" value="true" />
		<param name="$(arg name)/depth_i_publish_topic" value="$(arg enable_depth)" />
		<param name="$(arg name)/depth_i_width"
			value="$(eval depth_module_depth_profile.split(',')[0])" />
		<param name="$(arg name)/depth_i_height"
			value="$(eval depth_module_depth_profile.split(',')[1])" />
		<param name="$(arg name)/depth_i_fps"
			value="$(eval depth_module_depth_profile.split(',')[2])" />

		<param name="$(arg name)/infra1_i_publish_topic" value="$(arg enable_infra1)" />
		<param name="$(arg name)/infra1_i_width"
			value="$(eval depth_module_infra_profile.split(',')[0])" />
		<param name="$(arg name)/infra1_i_height"
			value="$(eval depth_module_infra_profile.split(',')[1])" />
		<param name="$(arg name)/infra1_i_fps"
			value="$(eval depth_module_infra_profile.split(',')[2])" />

		<param name="$(arg name)/infra2_i_publish_topic" value="$(arg enable_infra2)" />
		<param name="$(arg name)/infra2_i_width"
			value="$(eval depth_module_infra_profile.split(',')[0])" />
		<param name="$(arg name)/infra2_i_height"
			value="$(eval depth_module_infra_profile.split(',')[1])" />
		<param name="$(arg name)/infra2_i_fps"
			value="$(eval depth_module_infra_profile.split(',')[2])" />

		<param name="$(arg name)/depth_i_left_rect_publish_topic" value="true" />
		<param name="$(arg name)/depth_i_right_rect_publish_topic" value="true" />
	</group>

	<arg name="params_file" default="$(find depthai_ros_driver)/config/camera.yaml" />
	<arg name="camera_model" default="OAK-D" />

	<arg name="base_frame" default="oak-d_frame" />

	<arg name="cam_pos_x" default="0.0" />
	<!-- Position respect to base frame (i.e. "base_link) -->
	<arg name="cam_pos_y" default="0.0" />
	<!-- Position respect to base frame (i.e. "base_link) -->
	<arg name="cam_pos_z" default="0.0" />
	<!-- Position respect to base frame (i.e. "base_link) -->
	<arg name="cam_roll" default="0.0" />
	<!-- Orientation respect to base frame (i.e. "base_link) -->
	<arg name="cam_pitch" default="0.0" />
	<!-- Orientation respect to base frame (i.e. "base_link) -->
	<arg name="cam_yaw" default="0.0" />
	<!-- Orientation respect to base frame (i.e. "base_link) -->

	<param name="$(arg name)/camera_i_camera_model" value="$(arg camera_model)"
		if="$(arg override_cam_model)" />
	<param name="$(arg name)/camera_i_base_frame" value="$(arg base_frame)" />
	<param name="$(arg name)/camera_i_parent_frame" value="$(arg parent_frame)" />
	<param name="$(arg name)/camera_i_cam_pos_x" value="$(arg cam_pos_x)" />
	<param name="$(arg name)/camera_i_cam_pos_y" value="$(arg cam_pos_y)" />
	<param name="$(arg name)/camera_i_cam_pos_z" value="$(arg cam_pos_z)" />
	<param name="$(arg name)/camera_i_cam_roll" value="$(arg cam_roll)" />
	<param name="$(arg name)/camera_i_cam_pitch" value="$(arg cam_pitch)" />
	<param name="$(arg name)/camera_i_cam_yaw" value="$(arg cam_yaw)" />
	<param name="$(arg name)/camera_i_imu_from_descr" value="$(arg imu_from_descr)" />
	<param name="$(arg name)/camera_i_publish_tf_from_calibration"
		value="$(arg publish_tf_from_calibration)" />

	<group if="$(arg enable_pointcloud)">
		<param name="$(arg name)/pipeline_gen_i_enable_sync" value="true" />
		<param name="$(arg name)/rgb_i_synced" value="true" />
		<param name="$(arg name)/stereo_i_synced" value="true" />
	</group>

	<arg name="launch_prefix" default="" />


	<rosparam file="$(arg params_file)" />
	<node pkg="rosservice" if="$(optenv DEPTHAI_DEBUG 0)" type="rosservice" name="set_log_level"
		args="call --wait /oak_nodelet_manager/set_logger_level 'ros.depthai_ros_driver' 'debug'" />

	<include unless="$(arg publish_tf_from_calibration)"
		file="$(find depthai_descriptions)/launch/urdf.launch">
		<arg name="base_frame" value="$(arg  name)" />
		<arg name="parent_frame" value="$(arg  parent_frame)" />
		<arg name="camera_model" value="$(arg  camera_model)" />
		<arg name="tf_prefix" value="$(arg  name)" />
		<arg name="cam_pos_x" value="$(arg  cam_pos_x)" />
		<arg name="cam_pos_y" value="$(arg  cam_pos_y)" />
		<arg name="cam_pos_z" value="$(arg  cam_pos_z)" />
		<arg name="cam_roll" value="$(arg  cam_roll)" />
		<arg name="cam_pitch" value="$(arg  cam_pitch)" />
		<arg name="cam_yaw" value="$(arg  cam_yaw)" />
		<arg name="rs_compat" value="$(arg  rs_compat)" />
	</include>


	<node pkg="nodelet" type="nodelet" name="$(arg  name)_nodelet_manager"
		launch-prefix="$(arg launch_prefix)" args="manager" output="screen">
		<remap from="/nodelet_manager/load_nodelet" to="$(arg name)/nodelet_manager/load_nodelet" />
		<remap from="/nodelet_manager/unload_nodelet"
			to="$(arg name)/nodelet_manager/unload_nodelet" />
		<remap from="/nodelet_manager/list" to="$(arg name)/nodelet_manager/list" />
	</node>

	<node name="$(arg  name)" pkg="nodelet" type="nodelet" output="screen" required="true"
		args="load depthai_ros_driver/Camera $(arg  name)_nodelet_manager">
	</node>

	<node pkg="nodelet" type="nodelet" name="rectify_color"
		args="load image_proc/rectify $(arg  name)_nodelet_manager" if="$(arg rectify_rgb)">
		<remap from="image_mono" to="$(arg name)/$(arg color_sens_name)/image_raw" />
		<remap from="image_rect" to="$(arg name)/$(arg color_sens_name)/image_rect" />
	</node>

	<node pkg="nodelet" type="nodelet" name="depth_image_to_rgb_pointcloud"
		args="load depth_image_proc/point_cloud_xyzrgb $(arg  name)_nodelet_manager"
		if="$(arg enable_pointcloud)">
		<param name="queue_size" value="10" />

		<remap from="rgb/camera_info" to="$(arg name)/$(arg color_sens_name)/camera_info" />
		<remap from="rgb/image_rect_color" to="$(arg name)/$(arg color_sens_name)/image_rect" if="$(arg rectify_rgb)" />
		<remap from="rgb/image_rect_color" to="$(arg name)/$(arg color_sens_name)/image_raw"
			unless="$(arg rectify_rgb)" />
		<remap from="depth_registered/image_rect" to="$(arg name)/$(arg stereo_sens_name)/$(arg depth_topic_suffix)" />
		<remap from="depth_registered/points" to="$(arg points_topic_name)" />
	</node>

</launch>
```

### ROS 2 Foxy

```sh
source /opt/ros/foxy/setup.bash
source /home/user/dai_ros2_ws/install/setup.bash

ros2 launch depthai_ros_driver camera.launch.py
```

```py title="camera.launch.py"
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    log_level = 'info'
    if(context.environment.get('DEPTHAI_DEBUG')=='1'):
        log_level='debug'

    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')

    params_file = LaunchConfiguration("params_file")
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')

    name = LaunchConfiguration('name').perform(context)

    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    cam_pos_x    = LaunchConfiguration('cam_pos_x',     default = '0.0')
    cam_pos_y    = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z    = LaunchConfiguration('cam_pos_z',     default = '0.0')
    cam_roll     = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch    = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw      = LaunchConfiguration('cam_yaw',       default = '0.0')

    return [
            Node(
                condition=IfCondition(LaunchConfiguration("use_rviz").perform(context)),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urdf_launch_dir, 'urdf_launch.py')),
            launch_arguments={'tf_prefix': name,
                              'camera_model': camera_model,
                              'base_frame': name,
                              'parent_frame': parent_frame,
                              'cam_pos_x': cam_pos_x,
                              'cam_pos_y': cam_pos_y,
                              'cam_pos_z': cam_pos_z,
                              'cam_roll': cam_roll,
                              'cam_pitch': cam_pitch,
                              'cam_yaw': cam_yaw}.items()),

        ComposableNodeContainer(
            name=name+"_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name=name,
                        parameters=[params_file],
                    )
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output="both",
        ),

    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_prefix, 'config', 'camera.yaml')),
        DeclareLaunchArgument("use_rviz", default_value='false'),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(depthai_prefix, "config", "rviz", "rgbd.rviz"))
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
```

```yaml title="camera.yaml"
camera:
  i_enable_imu: true
  i_enable_ir: true
  i_floodlight_brightness: 0
  i_ip: ''
  i_laser_dot_brightness: 800
  i_mx_id: ''
  i_nn_type: spatial
  i_pipeline_type: RGBD
  i_usb_port_id: ''
  i_usb_speed: SUPER_PLUS
  i_pipeline_dump: false
  i_calibration_dump: false
  i_external_calibration_path: ''
left:
  i_board_socket_id: 1
  i_calibration_file: ''
  i_get_base_device_timestamp: true
  i_fps: 30.0
  i_height: 720
  i_low_bandwidth: false
  i_low_bandwidth_quality: 50
  i_max_q_size: 30
  i_publish_topic: false
  i_simulate_from_topic: false
  i_disable_node: false
  i_resolution: '720'
  i_width: 1280
  r_exposure: 1000
  r_iso: 800
  r_set_man_exposure: false
nn:
  i_nn_config_path: depthai_ros_driver/mobilenet
  i_disable_resize: false
  i_enable_passthrough: false
  i_enable_passthrough_depth: false
rgb:
  i_board_socket_id: 0
  i_simulate_from_topic: false
  i_get_base_device_timestamp: true
  i_disable_node: false
  i_calibration_file: ''
  i_enable_preview: false
  i_fps: 30.0
  i_height: 720
  i_interleaved: false
  i_keep_preview_aspect_ratio: true
  i_low_bandwidth: false
  i_low_bandwidth_quality: 50
  i_max_q_size: 30
  i_preview_size: 300
  i_publish_topic: true
  i_resolution: '1080'
  i_set_isp_scale: true
  i_isp_num: 2
  i_isp_den: 3
  i_output_isp: true
  i_width: 1280
  r_exposure: 20000
  r_focus: 1
  r_iso: 800
  r_set_man_exposure: false
  r_set_man_focus: false
  r_set_man_whitebalance: false
  r_whitebalance: 3300
right:
  i_board_socket_id: 2
  i_calibration_file: ''
  i_get_base_device_timestamp: true
  i_fps: 30.0
  i_height: 720
  i_low_bandwidth: false
  i_low_bandwidth_quality: 50
  i_max_q_size: 30
  i_publish_topic: false
  i_simulate_from_topic: false
  i_disable_node: false
  i_resolution: '720'
  i_width: 1280
  r_exposure: 1000
  r_iso: 800
  r_set_man_exposure: false
stereo:
  i_align_depth: true
  i_get_base_device_timestamp: true
  i_output_disparity: false
  i_bilateral_sigma: 0
  i_board_socket_id: 0
  i_depth_filter_size: 5
  i_depth_preset: HIGH_ACCURACY
  i_disparity_width: DISPARITY_96
  i_enable_companding: false
  i_enable_decimation_filter: false
  i_enable_distortion_correction: true
  i_enable_spatial_filter: false
  i_enable_speckle_filter: false
  i_enable_temporal_filter: false
  i_enable_threshold_filter: false
  i_extended_disp: false
  i_height: 720
  i_low_bandwidth: false
  i_low_bandwidth_quality: 50
  i_lr_check: true
  i_lrc_threshold: 10
  i_max_q_size: 30
  i_rectify_edge_fill_color: 0
  i_stereo_conf_threshold: 255
  i_set_input_size: false
  i_input_width: 1280
  i_input_height: 720
  i_width: 1280
use_sim_time: false
```
