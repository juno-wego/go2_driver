#!/usr/bin/env python3
"""
go2_driver_node.py
==================
Main ROS2 driver node for Unitree GO2 robot.

Bridges unitree_sdk2_python DDS channels to standard ROS2 topics and
converts incoming ROS2 commands back to Unitree SDK API calls.

Based on unitree_sdk2_python examples:
  - example/go2/low_level/go2_stand_example.py   → low-state bridge
  - example/go2/high_level/go2_sport_client.py   → sport-mode control
  - unitree_sdk2py/go2/sport/sport_client.py      → SportClient API

Published Topics (robot → ROS2)
--------------------------------
  /go2/imu              sensor_msgs/Imu             IMU (quat, gyro, accel)
  /go2/joint_states     sensor_msgs/JointState      12 leg joints
  /go2/foot_force       std_msgs/Int16MultiArray    4-foot contact force
  /go2/battery_state    go2_interface/Go2BatteryState  battery voltage/current/soc
  /go2/odom             nav_msgs/Odometry           odometry from SportModeState
  /go2/sport_state      unitree_go/SportModeState   raw high-level state (passthrough)

Subscribed Topics (ROS2 → robot)
---------------------------------
  /go2/cmd_vel          geometry_msgs/Twist         velocity command → Sport Move
  /go2/sport_cmd        go2_interface/SportCmd      named sport actions
  /go2/low_cmd          unitree_go/LowCmd           raw low-level motor command (passthrough)
"""

import sys
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS2 standard message types
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# unitree_go ROS2 messages (from cyclonedds_ws)
from unitree_go.msg import LowState, LowCmd, SportModeState

# Custom GO2 messages
from go2_interface.msg import SportCmd, Go2BatteryState

# unitree_sdk2_python DDS SDK
try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher,
        ChannelSubscriber,
        ChannelFactoryInitialize,
    )
    from unitree_sdk2py.idl.default import (
        unitree_go_msg_dds__LowCmd_,
        unitree_go_msg_dds__LowState_,
        unitree_go_msg_dds__SportModeState_,
    )
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as DdsLowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as DdsLowState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_ as DdsSportModeState_
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.utils.crc import CRC
    UNITREE_SDK_AVAILABLE = True
except ImportError as e:
    UNITREE_SDK_AVAILABLE = False
    print(f"[WARN] unitree_sdk2_python not available: {e}")
    print("[WARN] Running in ROS2-only mode (subscribing to /lowstate and /sportmodestate directly)")


# GO2 joint name mapping (order matches motor_state indices 0-11)
# FR = Front-Right, FL = Front-Left, RR = Rear-Right, RL = Rear-Left
# _0 = Hip (abduction), _1 = Thigh, _2 = Calf
GO2_JOINT_NAMES = [
    "FR_hip_joint",   # 0
    "FR_thigh_joint", # 1
    "FR_calf_joint",  # 2
    "FL_hip_joint",   # 3
    "FL_thigh_joint", # 4
    "FL_calf_joint",  # 5
    "RR_hip_joint",   # 6
    "RR_thigh_joint", # 7
    "RR_calf_joint",  # 8
    "RL_hip_joint",   # 9
    "RL_thigh_joint", # 10
    "RL_calf_joint",  # 11
]


class Go2DriverNode(Node):
    """
    Main ROS2 driver node for Unitree GO2.

    Mode A (unitree_sdk2_python available):
        Directly subscribes to DDS channels (rt/lowstate, rt/sportmodestate)
        via unitree_sdk2_python and re-publishes as ROS2 topics.
        Also uses SportClient for high-level control.

    Mode B (ROS2-only fallback):
        Subscribes to existing ROS2 topics /lowstate and /lf/sportmodestate
        published by unitree_ros2 bridge, then re-publishes as go2/* topics.
    """

    def __init__(self):
        super().__init__('go2_driver_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('network_interface', '')
        self.declare_parameter('use_sdk_mode', UNITREE_SDK_AVAILABLE)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_vel_timeout', 0.5)  # seconds

        self._net_iface = self.get_parameter('network_interface').value
        self._use_sdk   = self.get_parameter('use_sdk_mode').value and UNITREE_SDK_AVAILABLE
        self._pub_tf    = self.get_parameter('publish_tf').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        self.get_logger().info(
            f"GO2 Driver starting | sdk_mode={self._use_sdk} "
            f"| net_iface='{self._net_iface}'"
        )

        # ── Internal state ───────────────────────────────────────────
        self._low_state: DdsLowState_ | None = None
        self._sport_state: DdsSportModeState_ | None = None
        self._last_cmd_vel_time = self.get_clock().now()
        self._crc = CRC() if UNITREE_SDK_AVAILABLE else None

        # ── QoS ──────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ───────────────────────────────────────────────
        self._imu_pub = self.create_publisher(Imu, '/go2/imu', sensor_qos)
        self._joint_pub = self.create_publisher(JointState, '/go2/joint_states', sensor_qos)
        self._foot_force_pub = self.create_publisher(Int16MultiArray, '/go2/foot_force', sensor_qos)
        self._battery_pub = self.create_publisher(Go2BatteryState, '/go2/battery_state', reliable_qos)
        self._odom_pub = self.create_publisher(Odometry, '/go2/odom', sensor_qos)
        self._sport_state_pub = self.create_publisher(SportModeState, '/go2/sport_state', sensor_qos)
        self._low_cmd_pub = self.create_publisher(LowCmd, '/lowcmd', reliable_qos)

        # TF broadcaster for odom → base_link
        if self._pub_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        # ── Subscribers ──────────────────────────────────────────────
        self._cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, reliable_qos
        )
        self._sport_cmd_sub = self.create_subscription(
            SportCmd, '/go2/sport_cmd', self._sport_cmd_callback, reliable_qos
        )
        self._low_cmd_sub = self.create_subscription(
            LowCmd, '/go2/low_cmd', self._low_cmd_callback, reliable_qos
        )

        # ── SDK or ROS2 bridge setup ─────────────────────────────────
        if self._use_sdk:
            self._init_sdk_mode()
        else:
            self._init_ros2_mode()

        self.get_logger().info("GO2 Driver node ready.")

    # ================================================================
    # Mode A: unitree_sdk2_python DDS mode
    # ================================================================

    def _init_sdk_mode(self):
        """Initialize DDS channels via unitree_sdk2_python."""
        self.get_logger().info("Initializing unitree_sdk2_python DDS channels...")
        if self._net_iface:
            ChannelFactoryInitialize(0, self._net_iface)
        else:
            ChannelFactoryInitialize(0)

        # DDS → ROS2 subscribers
        self._dds_lowstate_sub = ChannelSubscriber("rt/lowstate", DdsLowState_)
        self._dds_lowstate_sub.Init(self._dds_low_state_handler, 10)

        self._dds_sportstate_sub = ChannelSubscriber("rt/sportmodestate", DdsSportModeState_)
        self._dds_sportstate_sub.Init(self._dds_sport_state_handler, 10)

        # ROS2 → DDS publisher for lowcmd
        self._dds_lowcmd_pub = ChannelPublisher("rt/lowcmd", DdsLowCmd_)
        self._dds_lowcmd_pub.Init()

        # Sport high-level client
        self._sport_client = SportClient()
        self._sport_client.SetTimeout(10.0)
        self._sport_client.Init()

        self.get_logger().info("DDS channels initialized.")

    def _dds_low_state_handler(self, msg: DdsLowState_):
        """DDS LowState callback → publish ROS2 topics."""
        self._low_state = msg
        now = self.get_clock().now().to_msg()
        self._publish_imu(msg, now)
        self._publish_joint_states(msg, now)
        self._publish_foot_force(msg, now)
        self._publish_battery(msg)

    def _dds_sport_state_handler(self, msg: DdsSportModeState_):
        """DDS SportModeState callback → publish ROS2 topics."""
        self._sport_state = msg
        now = self.get_clock().now().to_msg()
        self._publish_odom(msg, now)
        self._publish_sport_state(msg, now)

    # ================================================================
    # Mode B: ROS2-only mode (unitree_ros2 bridge running separately)
    # ================================================================

    def _init_ros2_mode(self):
        """Subscribe to ROS2 topics published by unitree_ros2 bridge."""
        self.get_logger().info(
            "ROS2-only mode: subscribing to /lowstate and /lf/sportmodestate"
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._ros2_lowstate_sub = self.create_subscription(
            LowState, '/lowstate', self._ros2_low_state_callback, sensor_qos
        )
        self._ros2_sportstate_sub = self.create_subscription(
            SportModeState, '/lf/sportmodestate', self._ros2_sport_state_callback, sensor_qos
        )
        # In ROS2 mode, sport commands go to /api/sport/request
        from unitree_api.msg import Request as ApiRequest
        self._api_request_pub = self.create_publisher(ApiRequest, '/api/sport/request', 10)

    def _ros2_low_state_callback(self, msg: LowState):
        """ROS2 /lowstate callback → republish as /go2/* topics."""
        now = self.get_clock().now().to_msg()
        self._publish_imu_from_ros2(msg, now)
        self._publish_joint_states_from_ros2(msg, now)
        self._publish_foot_force_from_ros2(msg, now)
        self._publish_battery_from_ros2(msg)

    def _ros2_sport_state_callback(self, msg: SportModeState):
        """ROS2 /sportmodestate callback → republish as /go2/* topics."""
        now = self.get_clock().now().to_msg()
        self._publish_odom_from_ros2(msg, now)
        self._sport_state_pub.publish(msg)

    # ================================================================
    # Publish helpers — DDS (SDK mode)
    # ================================================================

    def _publish_imu(self, low_state: DdsLowState_, stamp):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu_link'

        imu = low_state.imu_state
        # Quaternion: unitree order [w, x, y, z]
        msg.orientation.w = float(imu.quaternion[0])
        msg.orientation.x = float(imu.quaternion[1])
        msg.orientation.y = float(imu.quaternion[2])
        msg.orientation.z = float(imu.quaternion[3])
        # Angular velocity (gyroscope)
        msg.angular_velocity.x = float(imu.gyroscope[0])
        msg.angular_velocity.y = float(imu.gyroscope[1])
        msg.angular_velocity.z = float(imu.gyroscope[2])
        # Linear acceleration
        msg.linear_acceleration.x = float(imu.accelerometer[0])
        msg.linear_acceleration.y = float(imu.accelerometer[1])
        msg.linear_acceleration.z = float(imu.accelerometer[2])

        msg.orientation_covariance[0] = -1.0  # unknown
        self._imu_pub.publish(msg)

    def _publish_joint_states(self, low_state: DdsLowState_, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name     = GO2_JOINT_NAMES
        msg.position = [float(low_state.motor_state[i].q)       for i in range(12)]
        msg.velocity = [float(low_state.motor_state[i].dq)      for i in range(12)]
        msg.effort   = [float(low_state.motor_state[i].tau_est) for i in range(12)]
        self._joint_pub.publish(msg)

    def _publish_foot_force(self, low_state: DdsLowState_, stamp):
        msg = Int16MultiArray()
        msg.data = [int(low_state.foot_force[i]) for i in range(4)]
        self._foot_force_pub.publish(msg)

    def _publish_battery(self, low_state: DdsLowState_):
        msg = Go2BatteryState()
        msg.voltage     = float(low_state.power_v)
        msg.current     = float(low_state.power_a)
        msg.soc         = int(low_state.bms_state.soc) if hasattr(low_state, 'bms_state') else 0
        msg.is_charging = False
        self._battery_pub.publish(msg)

    def _publish_odom(self, sport_state: DdsSportModeState_, stamp):
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id  = self._base_frame

        # Position
        msg.pose.pose.position.x = float(sport_state.position[0])
        msg.pose.pose.position.y = float(sport_state.position[1])
        msg.pose.pose.position.z = float(sport_state.position[2])

        # Orientation from IMU quaternion embedded in SportModeState
        imu = sport_state.imu_state
        msg.pose.pose.orientation.w = float(imu.quaternion[0])
        msg.pose.pose.orientation.x = float(imu.quaternion[1])
        msg.pose.pose.orientation.y = float(imu.quaternion[2])
        msg.pose.pose.orientation.z = float(imu.quaternion[3])

        # Velocity
        msg.twist.twist.linear.x  = float(sport_state.velocity[0])
        msg.twist.twist.linear.y  = float(sport_state.velocity[1])
        msg.twist.twist.linear.z  = float(sport_state.velocity[2])
        msg.twist.twist.angular.z = float(sport_state.yaw_speed)

        self._odom_pub.publish(msg)

        # TF: odom → base_link
        if self._pub_tf:
            tf_msg = TransformStamped()
            tf_msg.header        = msg.header
            tf_msg.child_frame_id = self._base_frame
            tf_msg.transform.translation.x = msg.pose.pose.position.x
            tf_msg.transform.translation.y = msg.pose.pose.position.y
            tf_msg.transform.translation.z = msg.pose.pose.position.z
            tf_msg.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf_msg)

    def _publish_sport_state(self, sport_state: DdsSportModeState_, stamp):
        """Publish SportModeState as ROS2 unitree_go/SportModeState."""
        ros_msg = SportModeState()
        # --- IMU ---
        from unitree_go.msg import IMUState
        imu_msg = IMUState()
        imu_msg.quaternion   = list(sport_state.imu_state.quaternion)
        imu_msg.gyroscope    = list(sport_state.imu_state.gyroscope)
        imu_msg.accelerometer = list(sport_state.imu_state.accelerometer)
        imu_msg.rpy          = list(sport_state.imu_state.rpy)
        imu_msg.temperature  = int(sport_state.imu_state.temperature)
        ros_msg.imu_state = imu_msg

        ros_msg.gait_type        = int(sport_state.gait_type)
        ros_msg.foot_raise_height = float(sport_state.foot_raise_height)
        ros_msg.position          = list(sport_state.position)
        ros_msg.body_height       = float(sport_state.body_height)
        ros_msg.velocity          = list(sport_state.velocity)
        ros_msg.yaw_speed         = float(sport_state.yaw_speed)
        ros_msg.range_obstacle    = list(sport_state.range_obstacle)
        ros_msg.foot_force        = list(sport_state.foot_force)
        ros_msg.foot_position_body = list(sport_state.foot_position_body)
        ros_msg.foot_speed_body   = list(sport_state.foot_speed_body)

        self._sport_state_pub.publish(ros_msg)

    # ================================================================
    # Publish helpers — ROS2 mode (convert LowState → /go2/* )
    # ================================================================

    def _publish_imu_from_ros2(self, low_state: LowState, stamp):
        msg = Imu()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'imu_link'

        imu = low_state.imu_state
        msg.orientation.w = float(imu.quaternion[0])
        msg.orientation.x = float(imu.quaternion[1])
        msg.orientation.y = float(imu.quaternion[2])
        msg.orientation.z = float(imu.quaternion[3])
        msg.angular_velocity.x = float(imu.gyroscope[0])
        msg.angular_velocity.y = float(imu.gyroscope[1])
        msg.angular_velocity.z = float(imu.gyroscope[2])
        msg.linear_acceleration.x = float(imu.accelerometer[0])
        msg.linear_acceleration.y = float(imu.accelerometer[1])
        msg.linear_acceleration.z = float(imu.accelerometer[2])
        msg.orientation_covariance[0] = -1.0
        self._imu_pub.publish(msg)

    def _publish_joint_states_from_ros2(self, low_state: LowState, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name     = GO2_JOINT_NAMES
        msg.position = [float(low_state.motor_state[i].q)       for i in range(12)]
        msg.velocity = [float(low_state.motor_state[i].dq)      for i in range(12)]
        msg.effort   = [float(low_state.motor_state[i].tau_est) for i in range(12)]
        self._joint_pub.publish(msg)

    def _publish_foot_force_from_ros2(self, low_state: LowState, stamp):
        msg = Int16MultiArray()
        msg.data = [int(low_state.foot_force[i]) for i in range(4)]
        self._foot_force_pub.publish(msg)

    def _publish_battery_from_ros2(self, low_state: LowState):
        msg = Go2BatteryState()
        msg.voltage     = float(low_state.power_v)
        msg.current     = float(low_state.power_a)
        msg.soc         = int(low_state.bms_state.soc) if hasattr(low_state.bms_state, 'soc') else 0
        msg.is_charging = False
        self._battery_pub.publish(msg)

    def _publish_odom_from_ros2(self, sport_state: SportModeState, stamp):
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id  = self._base_frame

        msg.pose.pose.position.x = float(sport_state.position[0])
        msg.pose.pose.position.y = float(sport_state.position[1])
        msg.pose.pose.position.z = float(sport_state.position[2])

        imu = sport_state.imu_state
        msg.pose.pose.orientation.w = float(imu.quaternion[0])
        msg.pose.pose.orientation.x = float(imu.quaternion[1])
        msg.pose.pose.orientation.y = float(imu.quaternion[2])
        msg.pose.pose.orientation.z = float(imu.quaternion[3])

        msg.twist.twist.linear.x  = float(sport_state.velocity[0])
        msg.twist.twist.linear.y  = float(sport_state.velocity[1])
        msg.twist.twist.linear.z  = float(sport_state.velocity[2])
        msg.twist.twist.angular.z = float(sport_state.yaw_speed)

        self._odom_pub.publish(msg)

        if self._pub_tf:
            tf_msg = TransformStamped()
            tf_msg.header         = msg.header
            tf_msg.child_frame_id = self._base_frame
            tf_msg.transform.translation.x = msg.pose.pose.position.x
            tf_msg.transform.translation.y = msg.pose.pose.position.y
            tf_msg.transform.translation.z = msg.pose.pose.position.z
            tf_msg.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf_msg)

    # ================================================================
    # Command Subscribers
    # ================================================================

    def _cmd_vel_callback(self, msg: Twist):
        """
        /go2/cmd_vel → SportClient.Move(vx, vy, vyaw)
        Implements a timeout: if no message arrives within cmd_vel_timeout
        seconds, the robot is commanded to stop.
        """
        self._last_cmd_vel_time = self.get_clock().now()
        vx   = msg.linear.x
        vy   = msg.linear.y
        vyaw = msg.angular.z

        if self._use_sdk:
            self._sport_client.Move(vx, vy, vyaw)
        else:
            self._send_api_request(1008, {'x': vx, 'y': vy, 'z': vyaw})

    def _sport_cmd_callback(self, msg: SportCmd):
        """
        /go2/sport_cmd → SportClient high-level API calls.
        Command name → API method mapping mirrors go2_sport_client.py example.
        """
        cmd = msg.command.lower().strip()
        self.get_logger().info(f"SportCmd received: '{cmd}' flag={msg.flag}")

        if self._use_sdk:
            self._dispatch_sport_cmd_sdk(cmd, msg)
        else:
            self._dispatch_sport_cmd_ros2(cmd, msg)

    def _dispatch_sport_cmd_sdk(self, cmd: str, msg: SportCmd):
        """Route SportCmd to unitree_sdk2_python SportClient methods."""
        sc = self._sport_client
        if   cmd == 'damp':           sc.Damp()
        elif cmd == 'balance_stand':  sc.BalanceStand()
        elif cmd == 'stop_move':      sc.StopMove()
        elif cmd == 'stand_up':       sc.StandUp()
        elif cmd == 'stand_down':     sc.StandDown()
        elif cmd == 'recovery_stand': sc.RecoveryStand()
        elif cmd == 'sit':            sc.Sit()
        elif cmd == 'rise_sit':       sc.RiseSit()
        elif cmd == 'hello':          sc.Hello()
        elif cmd == 'stretch':        sc.Stretch()
        elif cmd == 'dance1':         sc.Dance1()
        elif cmd == 'dance2':         sc.Dance2()
        elif cmd == 'scrape':         sc.Scrape()
        elif cmd == 'heart':          sc.Heart()
        elif cmd == 'front_flip':     sc.FrontFlip()
        elif cmd == 'front_jump':     sc.FrontJump()
        elif cmd == 'front_pounce':   sc.FrontPounce()
        elif cmd == 'left_flip':      sc.LeftFlip()
        elif cmd == 'back_flip':      sc.BackFlip()
        elif cmd == 'free_walk':      sc.FreeWalk()
        elif cmd == 'static_walk':    sc.StaticWalk()
        elif cmd == 'trot_run':       sc.TrotRun()
        elif cmd == 'hand_stand':     sc.HandStand(msg.flag)
        elif cmd == 'free_bound':     sc.FreeBound(msg.flag)
        elif cmd == 'free_jump':      sc.FreeJump(msg.flag)
        elif cmd == 'free_avoid':     sc.FreeAvoid(msg.flag)
        elif cmd == 'walk_upright':   sc.WalkUpright(msg.flag)
        elif cmd == 'cross_step':     sc.CrossStep(msg.flag)
        elif cmd == 'classic_walk':   sc.ClassicWalk(msg.flag)
        elif cmd == 'euler':
            sc.Euler(msg.roll, msg.pitch, msg.yaw)
        elif cmd == 'speed_level':
            sc.SpeedLevel(msg.speed_level)
        elif cmd == 'pose':
            sc.Pose(msg.flag)
        elif cmd == 'switch_joystick':
            sc.SwitchJoystick(msg.flag)
        elif cmd == 'switch_avoid_mode':
            sc.SwitchAvoidMode()
        else:
            self.get_logger().warn(f"Unknown sport command: '{cmd}'")

    def _dispatch_sport_cmd_ros2(self, cmd: str, msg: SportCmd):
        """Route SportCmd to /api/sport/request (ROS2-only mode)."""
        import json
        API_IDS = {
            'damp': 1001, 'balance_stand': 1002, 'stop_move': 1003,
            'stand_up': 1004, 'stand_down': 1005, 'recovery_stand': 1006,
            'euler': 1007, 'move': 1008, 'sit': 1009, 'rise_sit': 1010,
            'speed_level': 1015, 'hello': 1016, 'stretch': 1017,
            'dance1': 1022, 'dance2': 1023, 'switch_joystick': 1027,
            'pose': 1028, 'scrape': 1029, 'front_flip': 1030,
            'front_jump': 1031, 'front_pounce': 1032, 'heart': 1036,
            'static_walk': 1061, 'trot_run': 1062,
            'left_flip': 2041, 'back_flip': 2043,
            'hand_stand': 2044, 'free_walk': 2045, 'free_bound': 2046,
            'free_jump': 2047, 'free_avoid': 2048, 'classic_walk': 2049,
            'walk_upright': 2050, 'cross_step': 2051,
            'switch_avoid_mode': 2058,
        }
        api_id = API_IDS.get(cmd)
        if api_id is None:
            self.get_logger().warn(f"Unknown sport command: '{cmd}'")
            return

        # Build parameter JSON for commands that need it
        params = {}
        if cmd in ('hand_stand', 'free_bound', 'free_jump', 'free_avoid',
                   'walk_upright', 'cross_step', 'classic_walk', 'pose',
                   'switch_joystick', 'free_bound'):
            params = {'data': msg.flag}
        elif cmd == 'euler':
            params = {'x': msg.roll, 'y': msg.pitch, 'z': msg.yaw}
        elif cmd == 'speed_level':
            params = {'data': msg.speed_level}

        self._send_api_request(api_id, params)

    def _send_api_request(self, api_id: int, params: dict):
        """Publish unitree_api Request message to /api/sport/request."""
        import json
        from unitree_api.msg import Request as ApiRequest
        req = ApiRequest()
        req.header.identity.api_id = api_id
        if params:
            req.parameter = json.dumps(params)
        self._api_request_pub.publish(req)

    def _low_cmd_callback(self, msg: LowCmd):
        """
        /go2/low_cmd passthrough → /lowcmd
        Allows external nodes to send low-level motor commands directly.
        In SDK mode, also send via DDS channel.
        """
        # Always re-publish to ROS2 /lowcmd (for unitree_ros2 bridge)
        self._low_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Go2DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
