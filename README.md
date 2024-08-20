# PX4 ODOMETRY

## BUILD

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:omeredemen/px4_odometry.git

# before build please be sure px4_msgs package is sourced on your system
colcon build

# after build
source ~/ros2_ws/install/setup.bash

```

## BRIEF

`px4_odom` package contains a node called `px4_odom_node` , some launch and config files.

px4_odom_node provides some features:

1. Subscribe the odometry topics and get odometry messages from VIO packages and convert them to `VehicleOdometry`messages that can be understandable by PX4.
2. Publish these messages to `/fmu/in/vehicle_visual_odometry` topic.
3. This node gets `VehicleOdometry` messages from PX4 and broadcast as `tf`

Also this package contains some launch files. This file can be used for starting `realsense2_camera` , `openvins` , `rtabmap_ros`  packages.

## EXAMPLES

Here list of launch files and their usage:

1. `px4_odom.launch.py` : this file starts `px4_odom_node` . Its config file’s name is config.yaml and here is its context
    
    ```yaml
    sensor_debug: true
    broadcast_tf: true
    vio_topic: "/ov_msckf/odomimu"
    px4_pose_topic: "/px4_odom/pose"
    
    ```
    
2. `rs_openvins.launch.py` : this launch file starts `realsense2_camera` and `ov_msckf` packages. A file path that has configurations for openvins needs to be passed in command line.
    
    ```bash
    ros2 launch px4_odom rs_openvins.launch.py openvins_config_path:=/home/omer/rs_d456_stereo/estimator_config.yaml
    
    ```
    
3. `rs_rtapmab.launch.py` : this launch file starts `realsense2_camera` and `rtabmap_ros` packages.
    
    ```bash
    ros2 launch px4_odom rs_rtabmap.launch.py
    
    ```
    
4. `rs.launch.py` : this file starts realsense2_camera with `config/rs_stereo.yaml`
    
    ```bash
    ros2 launch px4_odom rs.launch.py
    
    ```
    

## RESOURCES

- https://docs.openvins.com/getting-started.html

- https://github.com/introlab/rtabmap_ros/tree/ros2

- https://github.com/mzahana/px4_ros_com/blob/main/include/px4_ros_com/px4_ros.h

