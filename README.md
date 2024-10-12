# circut_ws
RTK cup & EUROBOT 2025 repo

Это код для пк, в папке проекта:

```
colcon build
source install/setup.sh
source /opt/ros/humble/setup.bash
ros2 run cv_basics img_subscriber
```

Во втором терминале:
(пакет image-transport-plugin нужно будет самостоятельно установить)
```
source install/setup.sh
source /opt/ros/humble/setup.bash
ros2 run image_transport republish  compressed raw --ros-args -r in/compressed:=/image_raw/compressed -r out:=/image_raw/uncomp
```

slam:
ros2 launch shesnar navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true
ros2 launch shesnar localization_launch.py map:=./my_room.yaml use_sim_time:=false

cameras:
ros2 run image_transport republish  compressed raw --ros-args -r in/compressed:=/image_raw/compressed -r out:=/image_raw/uncom
ros2 run cv_basics img_subscriber
ssh:
ros2 run v4l2_camera v4l2_camera_node
ros2 run image_transport republish raw in:=/image_raw  compressed out:=/image_compressed



rpi:
ros2 launch sllidar_ros2 sllidar_a3_launch.py
ros2 run govno_interface hw_interface

main:
ros2 launch shesnar rsp.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 run route_controller driver