# Narval Description 

## Description

Descriptions of robot should be implemented in `xacro` directory. 

`xacro` parse is performed automatically

## Launch

This package has two `*.launch.py` files. 

```bash
description.launch.py
display.launch.py
```

### description.launch.py

Creates `/robot_description` topic

#### Parameters

- `urdf_file_name`: name of xacro file with description implemented in `xacro` directory

### display.launch.py

Creates `/robot_description` using `description.launch.py` with default parameters and opens `rviz2` with configuration file.

#### Parameters

- `jsp_gui`: Flag to enable joint_state_publisher_gui
- `rviz_config`: Absolute path to rviz config file (default: `urdf.rviz`)