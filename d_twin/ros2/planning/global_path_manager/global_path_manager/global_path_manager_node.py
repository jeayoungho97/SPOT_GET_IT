#!/usr/bin/env python3
"""
global_path_manager_node.py

시작 시:
  맵 로드 → 로봇별 path set 생성 → 유효성 판정 → 선정 → 발행
이후 spin() 유지 (TRANSIENT_LOCAL 캐시 보존)

Publish:
  /planning/global_path/robot_01  (nav_msgs/Path)
  /planning/global_path/sim_02    (nav_msgs/Path)
  /planning/global_path/sim_03    (nav_msgs/Path)
  ※ 토픽은 map.yaml의 starts 키 기준으로 자동 생성
"""

import math
import os
from typing import Dict

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from .path_set import GlobalPath, MapConfig, build_path_set, select_paths, validate_path_set


def _yaw_between(p1, p2) -> float:
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


def _to_path_msg(path: GlobalPath, stamp, frame_id: str) -> Path:
    msg = Path()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    wps = path.waypoints
    for i, (x, y) in enumerate(wps):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.05
        yaw = (_yaw_between(wps[i], wps[i+1]) if i < len(wps)-1
               else _yaw_between(wps[i-1], wps[i]) if len(wps) > 1 else 0.0)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.poses.append(pose)
    return msg


class GlobalPathManagerNode(Node):

    def __init__(self):
        super().__init__("global_path_manager_node")

        self.declare_parameter("frame_id",       "map")
        self.declare_parameter("map_config_path", "")

        self.frame_id      = self.get_parameter("frame_id").value
        map_config_path    = self.get_parameter("map_config_path").value

        if not map_config_path:
            map_config_path = os.path.join(
                get_package_share_directory("global_path_manager"),
                "config", "map.yaml",
            )

        # ── Publisher: TRANSIENT_LOCAL ──
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── 맵 로드 ──
        self.get_logger().info(f"맵 로드: {map_config_path}")
        cfg = MapConfig(map_config_path)
        self.get_logger().info(
            f"로봇별 출발지: { {k: v for k, v in cfg.starts.items()} }"
        )
        self.get_logger().info(
            f"장애물 {len(cfg.obstacles)}개: {[o['id'] for o in cfg.obstacles]}"
        )

        # 로봇별 publisher 생성
        pubs: Dict[str, rclpy.publisher.Publisher] = {}
        for robot_key in cfg.starts:
            topic = f"/planning/global_path/{robot_key}"
            pubs[robot_key] = self.create_publisher(Path, topic, qos)
            self.get_logger().info(f"Publisher 등록: {topic}")

        # ── path set 생성 ──
        path_sets = build_path_set(cfg)
        for robot_key, paths in path_sets.items():
            self.get_logger().info(f"  [{robot_key}] path set: {len(paths)}개")

        # ── 유효성 + 커버리지 판정 ──
        validate_path_set(path_sets, cfg)
        for robot_key, paths in path_sets.items():
            valid = sum(1 for p in paths if p.valid)
            self.get_logger().info(f"  [{robot_key}] 유효: {valid}/{len(paths)}개")

        # ── 선정 ──
        selected = select_paths(path_sets, starts=cfg.starts)

        # ── 발행 ──
        stamp = self.get_clock().now().to_msg()
        for robot_key, path in selected.items():
            pubs[robot_key].publish(_to_path_msg(path, stamp, self.frame_id))
            self.get_logger().info(
                f"  [{robot_key}] {path.path_id}  waypoints: {path.waypoints}"
            )

        self.get_logger().info("발행 완료.")


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
