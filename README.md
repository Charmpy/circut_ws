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