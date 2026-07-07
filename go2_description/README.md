# go2_description

ROS 2 description package for the Unitree GO2 robot.

Included assets:
- `urdf/` exported URDF
- `xacro/` source xacro files
- `meshes/` and `dae/` mesh assets
- `launch/go2_description.launch.py` for RViz and `robot_state_publisher`

This workspace copy is ROS 2 only. Legacy XML launch files were removed.

## When used for Isaac Gym or similar engines

Collision parameters in urdf can be amended to better train the robot:

Open "go2_description.urdf" in "./go2_description/urdf",
and amend the ` box size="0.213 0.0245 0.034" ` in links of "FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh".

For example, change previous values to ` box size="0.11 0.0245 0.034" ` means the length of the thigh is shortened from 0.213 to 0.11, which can avoid unnecessary collision between the thigh link and the calf link. 

The collision model before and after the above amendment are shown as "Normal_collision_model.png" and "Amended_collision_model.png" respectively.

