# Intel RealSense L515

## Links

- [Intel RealSense L515](https://www.intelrealsense.com/lidar-camera-l515/)
- [Intel RealSense Documentation](https://dev.intelrealsense.com/docs/docs-get-started)

## Documents

- [Intel RealSense L515 Datasheet](../../../assets/sensors/rgbd/intel_realsense_l515/intel_realsense_l515_datasheet.pdf)
- [Intel RealSense L515 3D Model (STEP)](../../../assets/sensors/rgbd/intel_realsense_l515/intel_realsense_l515_3d-model.stp)

## Setup

### ROS 1 Noetic

```sh
sudo apt install ros-noetic-realsense2-camera ros-noetic-realsense2-description
```

### ROS 2 Foxy

```sh
sudo apt install ros-foxy-realsense2-camera ros-foxy-realsense2-description
```

## Launch

### ROS 1 Noetic

```sh
source /opt/ros/noetic/setup.bash

roslaunch realsense2_camera rs_camera.launch
```

```xml title="rs_camera.launch"
<launch>

  <arg name="serial_no"           default=""/>
  <arg name="usb_port_id"         default=""/>
  <arg name="device_type"         default=""/>
  <arg name="json_file_path"      default=""/>
  <arg name="camera"              default="camera"/>
  <arg name="tf_prefix"           default="$(arg camera)"/>
  <arg name="external_manager"    default="false"/>
  <arg name="manager"             default="realsense2_camera_manager"/>
  <arg name="output"              default="screen"/>
  <arg name="respawn"             default="false"/>

  <arg name="fisheye_width"       default="-1"/>
  <arg name="fisheye_height"      default="-1"/>
  <arg name="enable_fisheye"      default="false"/>

  <arg name="depth_width"         default="-1"/>
  <arg name="depth_height"        default="-1"/>
  <arg name="enable_depth"        default="true"/>

  <arg name="confidence_width"    default="-1"/>
  <arg name="confidence_height"   default="-1"/>
  <arg name="enable_confidence"   default="true"/>
  <arg name="confidence_fps"      default="-1"/>

  <arg name="infra_width"         default="848"/>
  <arg name="infra_height"        default="480"/>
  <arg name="enable_infra"        default="false"/>
  <arg name="enable_infra1"       default="false"/>
  <arg name="enable_infra2"       default="false"/>
  <arg name="infra_rgb"           default="false"/>

  <arg name="color_width"         default="-1"/>
  <arg name="color_height"        default="-1"/>
  <arg name="enable_color"        default="true"/>

  <arg name="fisheye_fps"         default="-1"/>
  <arg name="depth_fps"           default="-1"/>
  <arg name="infra_fps"           default="30"/>
  <arg name="color_fps"           default="-1"/>
  <arg name="gyro_fps"            default="-1"/>
  <arg name="accel_fps"           default="-1"/>
  <arg name="enable_gyro"         default="false"/>
  <arg name="enable_accel"        default="false"/>

  <arg name="enable_pointcloud"         default="false"/>
  <arg name="pointcloud_texture_stream" default="RS2_STREAM_COLOR"/>
  <arg name="pointcloud_texture_index"  default="0"/>
  <arg name="allow_no_texture_points"   default="false"/>
  <arg name="ordered_pc"                default="false"/>

  <arg name="enable_sync"               default="false"/>
  <arg name="align_depth"               default="false"/>

  <arg name="publish_tf"                default="true"/>
  <arg name="tf_publish_rate"           default="0"/>

  <arg name="filters"                   default=""/>
  <arg name="clip_distance"             default="-2"/>
  <arg name="linear_accel_cov"          default="0.01"/>
  <arg name="initial_reset"             default="false"/>
  <arg name="reconnect_timeout"         default="6.0"/>
  <arg name="wait_for_device_timeout"   default="-1.0"/>
  <arg name="unite_imu_method"          default=""/>
  <arg name="topic_odom_in"             default="odom_in"/>
  <arg name="calib_odom_file"           default=""/>
  <arg name="publish_odom_tf"           default="true"/>

  <arg name="stereo_module/exposure/1"  default="7500"/>
  <arg name="stereo_module/gain/1"      default="16"/>
  <arg name="stereo_module/exposure/2"  default="1"/>
  <arg name="stereo_module/gain/2"      default="16"/>



  <group ns="$(arg camera)">

    <include file="$(find realsense2_camera)/launch/includes/nodelet.launch.xml">

      <arg name="tf_prefix"                value="$(arg tf_prefix)"/>
      <arg name="external_manager"         value="$(arg external_manager)"/>
      <arg name="manager"                  value="$(arg manager)"/>
      <arg name="output"                   value="$(arg output)"/>
      <arg name="respawn"                  value="$(arg respawn)"/>
      <arg name="serial_no"                value="$(arg serial_no)"/>
      <arg name="usb_port_id"              value="$(arg usb_port_id)"/>
      <arg name="device_type"              value="$(arg device_type)"/>
      <arg name="json_file_path"           value="$(arg json_file_path)"/>

      <arg name="enable_pointcloud"        value="$(arg enable_pointcloud)"/>
      <arg name="pointcloud_texture_stream"value="$(arg pointcloud_texture_stream)"/>
      <arg name="pointcloud_texture_index" value="$(arg pointcloud_texture_index)"/>
      <arg name="enable_sync"              value="$(arg enable_sync)"/>
      <arg name="align_depth"              value="$(arg align_depth)"/>

      <arg name="fisheye_width"            value="$(arg fisheye_width)"/>
      <arg name="fisheye_height"           value="$(arg fisheye_height)"/>
      <arg name="enable_fisheye"           value="$(arg enable_fisheye)"/>

      <arg name="depth_width"              value="$(arg depth_width)"/>
      <arg name="depth_height"             value="$(arg depth_height)"/>
      <arg name="enable_depth"             value="$(arg enable_depth)"/>

      <arg name="confidence_width"         value="$(arg confidence_width)"/>
      <arg name="confidence_height"        value="$(arg confidence_height)"/>
      <arg name="enable_confidence"        value="$(arg enable_confidence)"/>
      <arg name="confidence_fps"           value="$(arg confidence_fps)"/>

      <arg name="color_width"              value="$(arg color_width)"/>
      <arg name="color_height"             value="$(arg color_height)"/>
      <arg name="enable_color"             value="$(arg enable_color)"/>

      <arg name="infra_width"              value="$(arg infra_width)"/>
      <arg name="infra_height"             value="$(arg infra_height)"/>
      <arg name="enable_infra"             value="$(arg enable_infra)"/>
      <arg name="enable_infra1"            value="$(arg enable_infra1)"/>
      <arg name="enable_infra2"            value="$(arg enable_infra2)"/>
      <arg name="infra_rgb"                value="$(arg infra_rgb)"/>

      <arg name="fisheye_fps"              value="$(arg fisheye_fps)"/>
      <arg name="depth_fps"                value="$(arg depth_fps)"/>
      <arg name="infra_fps"                value="$(arg infra_fps)"/>
      <arg name="color_fps"                value="$(arg color_fps)"/>
      <arg name="gyro_fps"                 value="$(arg gyro_fps)"/>
      <arg name="accel_fps"                value="$(arg accel_fps)"/>
      <arg name="enable_gyro"              value="$(arg enable_gyro)"/>
      <arg name="enable_accel"             value="$(arg enable_accel)"/>

      <arg name="publish_tf"               value="$(arg publish_tf)"/>
      <arg name="tf_publish_rate"          value="$(arg tf_publish_rate)"/>

      <arg name="filters"                  value="$(arg filters)"/>
      <arg name="clip_distance"            value="$(arg clip_distance)"/>
      <arg name="linear_accel_cov"         value="$(arg linear_accel_cov)"/>
      <arg name="initial_reset"            value="$(arg initial_reset)"/>
      <arg name="reconnect_timeout"        value="$(arg reconnect_timeout)"/>
      <arg name="wait_for_device_timeout"  value="$(arg wait_for_device_timeout)"/>
      <arg name="unite_imu_method"         value="$(arg unite_imu_method)"/>
      <arg name="topic_odom_in"            value="$(arg topic_odom_in)"/>
      <arg name="calib_odom_file"          value="$(arg calib_odom_file)"/>
      <arg name="publish_odom_tf"          value="$(arg publish_odom_tf)"/>
      <arg name="stereo_module/exposure/1" value="$(arg stereo_module/exposure/1)"/>
      <arg name="stereo_module/gain/1"     value="$(arg stereo_module/gain/1)"/>
      <arg name="stereo_module/exposure/2" value="$(arg stereo_module/exposure/2)"/>
      <arg name="stereo_module/gain/2"     value="$(arg stereo_module/gain/2)"/>

      <arg name="allow_no_texture_points"  value="$(arg allow_no_texture_points)"/>
      <arg name="ordered_pc"               value="$(arg ordered_pc)"/>

    </include>

  </group>

</launch>
```

### ROS 2 Foxy


```sh
source /opt/ros/foxy/setup.bash

ros2 launch realsense2_camera rs_launch.py
```

```py title="rplidar_s2_launch.py"
# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.

"""Launch realsense2_camera node."""
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


configurable_parameters = [{'name': 'camera_name',                   'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'serial_no',                     'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                   'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                   'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                   'default': "''", 'description': 'yaml config file'},
                           {'name': 'unite_imu_method',              'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'json_file_path',                'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'log_level',                     'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                        'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'depth_module.profile',          'default': '0,0,0', 'description': 'depth module profile'},
                           {'name': 'enable_depth',                  'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'rgb_camera.profile',            'default': '0,0,0', 'description': 'color image width'},
                           {'name': 'enable_color',                  'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_infra1',                 'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                 'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                     'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'tracking_module.profile',       'default': '0,0,0', 'description': 'fisheye width'},
                           {'name': 'enable_fisheye1',               'default': 'true', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2',               'default': 'true', 'description': 'enable fisheye2 stream'},
                           {'name': 'enable_confidence',             'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'gyro_fps',                      'default': '0', 'description': "''"},
                           {'name': 'accel_fps',                     'default': '0', 'description': "''"},
                           {'name': 'enable_gyro',                   'default': 'false', 'description': "''"},
                           {'name': 'enable_accel',                  'default': 'false', 'description': "''"},
                           {'name': 'enable_pose',                   'default': 'true', 'description': "''"},
                           {'name': 'pose_fps',                      'default': '200', 'description': "''"},
                           {'name': 'pointcloud.enable',             'default': 'false', 'description': ''},
                           {'name': 'pointcloud.stream_filter',      'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'enable_sync',                   'default': 'false', 'description': "''"},
                           {'name': 'align_depth.enable',            'default': 'false', 'description': "''"},
                           {'name': 'colorizer.enable',              'default': 'false', 'description': "''"},
                           {'name': 'clip_distance',                 'default': '-2.', 'description': "''"},
                           {'name': 'linear_accel_cov',              'default': '0.01', 'description': "''"},
                           {'name': 'initial_reset',                 'default': 'false', 'description': "''"},
                           {'name': 'allow_no_texture_points',       'default': 'false', 'description': "''"},
                           {'name': 'ordered_pc',                    'default': 'false', 'description': ''},
                           {'name': 'calib_odom_file',               'default': "''", 'description': "''"},
                           {'name': 'topic_odom_in',                 'default': "''", 'description': 'topic for T265 wheel odometry'},
                           {'name': 'tf_publish_rate',               'default': '0.0', 'description': 'Rate of publishing static_tf'},
                           {'name': 'diagnostics_period',            'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'decimation_filter.enable',      'default': 'false', 'description': 'Rate of publishing static_tf'},
                           {'name': 'rosbag_filename',               'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'depth_module.exposure.1',       'default': '7500', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'depth_module.gain.1',           'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'depth_module.exposure.2',       'default': '1', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'depth_module.gain.2',           'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'wait_for_device_timeout',       'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',             'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    log_level = 'info'
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
            # Realsense
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " == ''"])),
                package='realsense2_camera',
                node_namespace=LaunchConfiguration("camera_name"),
                node_name=LaunchConfiguration("camera_name"),
                node_executable='realsense2_camera_node',
                prefix=['stdbuf -o L'],
                parameters=[set_configurable_parameters(configurable_parameters)
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                ),
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " != ''"])),
                package='realsense2_camera',
                node_namespace=LaunchConfiguration("camera_name"),
                node_name=LaunchConfiguration("camera_name"),
                node_executable='realsense2_camera_node',
                prefix=['stdbuf -o L'],
                parameters=[set_configurable_parameters(configurable_parameters)
                            , PythonExpression([LaunchConfiguration("config_file")])
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                ),
            ])
    else:
        return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
            # Realsense
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " == ''"])),
                package='realsense2_camera',
                namespace=LaunchConfiguration("camera_name"),
                name=LaunchConfiguration("camera_name"),
                executable='realsense2_camera_node',
                parameters=[set_configurable_parameters(configurable_parameters)
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                emulate_tty=True,
                ),
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " != ''"])),
                package='realsense2_camera',
                namespace=LaunchConfiguration("camera_name"),
                name=LaunchConfiguration("camera_name"),
                executable='realsense2_camera_node',
                parameters=[set_configurable_parameters(configurable_parameters)
                            , PythonExpression([LaunchConfiguration("config_file")])
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                emulate_tty=True,
                ),
        ])
```
