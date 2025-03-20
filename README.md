# Marines-Jet-Packages
Nvidia Jetson submodule for [ROS project](https://github.com/PFlak/ROS2-MARINES)



## Packages

### [narval_description](./narval_description/README.md)

Description of robot

### [narval_PID](./narval_PID/README.md)

Implementation for PID controller for ROV robot

### [narval_stonefish](./narval_stonefish/README.md)

Simulation environment based on [stonefish repository](https://github.com/patrykcieslak/stonefish)

### [narval_thruster_manager](./narval_thruster_manager/README.md)

Thruster manager for Narval ROV


# Commanges
## Run camera node
``ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" -p camera_frame_id:=camera_optical_link -p video_device:="/dev/video2"``

