#!/bin/bash
timeout --preserve-status --foreground 30 ros2 bag record /camera_centre/camera_info /camera_centre/image /os_dome/metadata /os_dome/points /os_top/metadata /os_top/points /tf_static
