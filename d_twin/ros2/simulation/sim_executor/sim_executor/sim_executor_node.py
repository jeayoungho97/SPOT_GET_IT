#!/usr/bin/env python3
"""
sim_executor_node.py

역할:
  - /planning/global_path/sim_0{N} 수신 (TRANSIENT_LOCAL)
  - 현재 위치(start)에서 waypoint 순서대로 speed(m/s)로 이동
  - 매 tick마다 /localization/robot/state 발행
  - waypoint 도착 판정: arrival_dist(m) 이내

Subscribe:
  /planning/global_path/sim_02 or sim_03  (nav_msgs/Path)

Publish:
  /localization/robot/state  (robot_interfaces/RobotLocalization)
"""

import math
from typing import List, Tuple

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from robot_interfaces.msg import RobotLocalization

Waypoint = Tuple[float, float]

_ROBOT_SUFFIX = {2: "sim_02", 3: "sim_03"}


class SimExecutorNode(Node):

    def __init__(self):
        super().__init__("sim_executor_node")

        self.declare_parameter("robot_id",     2)
        self.declare_parameter("start_x",      2.0)
        self.declare_parameter("start_y",      2.0)
        self.declare_parameter("speed",        0.1)
        self.declare_parameter("arrival_dist", 0.7)
        self.declare_parameter("tick_rate",    20.0)
        self.declare_parameter("frame_id",     "map")

        self._robot_id   = self.get_parameter("robot_id").value
        self._x          = float(self.get_parameter("start_x").value)
        self._y          = float(self.get_parameter("start_y").value)
        self._speed      = float(self.get_parameter("speed").value)
        self._arr_dist   = float(self.get_parameter("arrival_dist").value)
        tick_rate        = float(self.get_parameter("tick_rate").value)
        self._frame_id   = self.get_parameter("frame_id").value

        self._yaw:         float         = 0.0
        self._waypoints:   List[Waypoint] = []
        self._wp_index:    int            = 0
        self._path_active: bool           = False
        self._dt = 1.0 / tick_rate

        suffix = _ROBOT_SUFFIX.get(self._robot_id, f"sim_0{self._robot_id}")

        # ── Subscriber: TRANSIENT_LOCAL ──
        sub_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Path,
            f"/planning/global_path/{suffix}",
            self._on_path,
            sub_qos,
        )

        # ── Publisher ──
        self._pub = self.create_publisher(
            RobotLocalization, "/localization/robot/state", 10
        )

        # ── tick 타이머 ──
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"sim_executor 시작: robot_id={self._robot_id} suffix={suffix} "
            f"start=({self._x:.1f},{self._y:.1f}) "
            f"speed={self._speed}m/s arrival_dist={self._arr_dist}m"
        )

    # ── 경로 수신 ─────────────────────────────────────────
    def _on_path(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("빈 path 수신, 무시")
            return

        self._waypoints = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        self._wp_index    = 0
        self._path_active = True

        self.get_logger().info(
            f"path 수신: {len(self._waypoints)}개 waypoints"
            f" → wp0={self._waypoints[0]}"
        )

    # ── tick ─────────────────────────────────────────────
    def _tick(self):
        if self._path_active:
            self._move()
        self._publish()

    def _move(self):
        if self._wp_index >= len(self._waypoints):
            if self._path_active:
                self._path_active = False
                self.get_logger().info("경로 완료.")
            return

        tx, ty = self._waypoints[self._wp_index]
        dx = tx - self._x
        dy = ty - self._y
        dist = math.hypot(dx, dy)

        if dist <= self._arr_dist:
            self.get_logger().info(
                f"wp{self._wp_index} 도착 ({tx:.1f},{ty:.1f})"
                f" → wp{self._wp_index + 1}"
            )
            self._wp_index += 1
            return

        step = self._speed * self._dt
        ratio = min(step / dist, 1.0)
        self._x   += dx * ratio
        self._y   += dy * ratio
        self._yaw  = math.atan2(dy, dx)

    def _publish(self):
        msg = RobotLocalization()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.robot_id        = self._robot_id
        msg.x               = self._x
        msg.y               = self._y
        msg.yaw             = self._yaw
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
