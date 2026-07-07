# go2_driver

ROS 2 workspace for the Unitree GO2 driver.

`go2_base` now follows the same bringup pattern as `b2_base`: a single launch
entrypoint with separate C++ control and state bridges on top of Unitree ROS 2
topics and `/api/sport/request`.

The shared `unitree_go` and `unitree_api` ROS 2 interface packages are used
from the local vendored copy in `/home/juno/ros2_ws/src/unitree_ros2_vendor`,
not directly from `/home/juno/ros2_ws/src/unitree`.

Useful helper:

```bash
sudo /home/juno/ros2_ws/src/go2_driver/go2_base/scripts/set_unitree_static_ip.sh enp3s0
ros2 launch go2_base go2_bringup.launch.py network_interface:=enp3s0
```

The static IP helper follows the Unitree ROS 2 docs default and sets the PC-side
address to `192.168.123.99/24` unless you override it.
