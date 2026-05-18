#!/usr/bin/env python3
"""
sim_executor_node.py
====================
프로젝트 : SPOT Get IT
작성일   : 2026-05-18
위치     : robot_ws/src/simulation/sim_executor/sim_executor/sim_executor_node.py

spot_02 / spot_03 시뮬 데이터를 BridgeDaemon(RPi5)으로 직접 UDP 송신.
ROS2 토픽으로 올리면 Jetson이 의도치 않게 수신하므로 토픽 사용 안 함.

기능
  1. /planning/global_path/spot_02|03 수신 → 경로 추종
  2. Pose   → PKT_TYPE_ODOM  (0x03)  UDP 직송  (10 Hz)
  3. MJPEG  → PKT_TYPE_IMAGE (0x01)  UDP 직송  (15 fps, 루프 재생)
  4. person_detected → PKT_TYPE_EVENT (0x07) UDP 직송  (키보드 수동 트리거)

UDP 프로토콜 (proto.h 기준, little-endian)
  PktHeader 24B
    type(1)  robot_id(1)  frag_idx(2)  frag_total(2)
    payload_len(2)  frame_id(4)  payload_offset(4)  timestamp_us(8)

  OdomPayload 24B  — x(f) y(f) theta(f) vx(f) vy(f) omega(f)
  EventPayload 104B — severity(B) reserved(B) event_type(H) code(I) message[96s]
  IMAGE: PROTO_MTU(1400B) 단위 프래그먼트

robot_id 체계 (0-based, pose_path_event_sender.yaml 동일)
  spot_01 = 0  (Jetson/oak_camera_driver 담당)
  spot_02 = 1
  spot_03 = 2

키보드 입력 (stdin, 줄 단위)
  s     시작 (경로 추종 + pose/영상 송출)
  v2    spot_02 탐지 ON
  f2    spot_02 탐지 OFF
  v3    spot_03 탐지 ON
  f3    spot_03 탐지 OFF
  q     종료
"""

import math
import socket
import struct
import sys
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from robot_interfaces.msg import GlobalPathWaypoints

# ─── proto.h 상수 ─────────────────────────────────────────────────────────────
PKT_TYPE_IMAGE = 0x01
PKT_TYPE_ODOM  = 0x03
PKT_TYPE_EVENT = 0x07

BRIDGE_PORT = 9000
PROTO_MTU   = 1400

EVENT_SEVERITY_CRITICAL    = 4
EVENT_TYPE_VICTIM_DETECTED = 8

# PktHeader: little-endian, 24B
# type(B) robot_id(B) frag_idx(H) frag_total(H) payload_len(H)
# frame_id(I) payload_offset(I) timestamp_us(Q)
_HDR_FMT  = '<BBHHHIIQ'
_HDR_SIZE = struct.calcsize(_HDR_FMT)   # 24

# OdomPayload: little-endian, 24B — x y theta vx vy omega
_ODOM_FMT  = '<ffffff'
_ODOM_SIZE = struct.calcsize(_ODOM_FMT)  # 24

# EventPayload: little-endian, 104B — severity reserved event_type code message[96]
_EVENT_FMT  = '<BBHi96s'
_EVENT_SIZE = struct.calcsize(_EVENT_FMT)  # 104


def _now_us() -> int:
    return int(time.monotonic() * 1_000_000)


def _hdr(pkt_type: int, robot_id: int, payload_len: int,
         frame_id: int, frag_idx: int = 0, frag_total: int = 1,
         offset: int = 0) -> bytes:
    return struct.pack(
        _HDR_FMT,
        pkt_type, robot_id,
        frag_idx, frag_total, payload_len,
        frame_id, offset,
        _now_us(),
    )


def _build_odom(robot_id: int, frame_id: int,
                x: float, y: float, theta: float) -> bytes:
    hdr = _hdr(PKT_TYPE_ODOM, robot_id, _ODOM_SIZE, frame_id)
    payload = struct.pack(_ODOM_FMT, x, y, theta, 0.0, 0.0, 0.0)
    return hdr + payload


def _build_event(robot_id: int, frame_id: int, detected: bool) -> bytes:
    hdr = _hdr(PKT_TYPE_EVENT, robot_id, _EVENT_SIZE, frame_id)
    code    = 1 if detected else 0
    msg_str = b'person detected' if detected else b'person cleared'
    payload = struct.pack(_EVENT_FMT,
                          EVENT_SEVERITY_CRITICAL, 0,
                          EVENT_TYPE_VICTIM_DETECTED,
                          code,
                          msg_str)
    return hdr + payload


def _build_image_fragments(robot_id: int, frame_id: int,
                           jpeg: bytes) -> list[bytes]:
    size       = len(jpeg)
    frag_total = math.ceil(size / PROTO_MTU)
    packets    = []
    for idx in range(frag_total):
        offset  = idx * PROTO_MTU
        chunk   = jpeg[offset: offset + PROTO_MTU]
        hdr     = _hdr(PKT_TYPE_IMAGE, robot_id, len(chunk),
                       frame_id, idx, frag_total, offset)
        packets.append(hdr + chunk)
    return packets


# ─── 단일 로봇 시뮬 상태 ──────────────────────────────────────────────────────
class RobotSim:

    def __init__(self, ros_id: str, udp_id: int,
                 video_path: str, speed: float,
                 quality: int, max_size: int):
        self.ros_id    = ros_id
        self.udp_id    = udp_id          # proto.h robot_id (0-based)
        self.speed     = speed
        self._quality  = quality
        self._max_size = max_size

        # 경로 추종
        self._waypoints: list[tuple[float, float]] = []
        self._seg     = 0
        self._prog    = 0.0
        self._cur_x   = 0.0
        self._cur_y   = 0.0
        self._cur_yaw = 0.0

        # 시퀀스 번호
        self.odom_seq  = 0
        self.event_seq = 0
        self.img_seq   = 0

        # rate limit (odom)
        self._last_odom_us = 0

        # 영상
        self._cap = cv2.VideoCapture(video_path)

    # ── 경로 ─────────────────────────────────────────────────────────────────
    def set_path(self, waypoints_msg: list) -> None:
        self._waypoints = [(wp.x_m, wp.y_m) for wp in waypoints_msg]
        self._seg  = 0
        self._prog = 0.0
        if self._waypoints:
            self._cur_x, self._cur_y = self._waypoints[0]

    def has_path(self) -> bool:
        return len(self._waypoints) >= 2

    def advance(self, dt: float) -> tuple[float, float, float]:
        if not self.has_path():
            return self._cur_x, self._cur_y, self._cur_yaw

        wps = self._waypoints
        n   = len(wps)

        if self._seg >= n - 1:
            self._cur_x, self._cur_y = wps[-1]
            return self._cur_x, self._cur_y, self._cur_yaw

        p0 = wps[self._seg]
        p1 = wps[self._seg + 1]
        dx      = p1[0] - p0[0]
        dy      = p1[1] - p0[1]
        seg_len = math.hypot(dx, dy) or 1e-6

        self._prog += self.speed * dt
        while self._prog >= seg_len and self._seg < n - 2:
            self._prog -= seg_len
            self._seg  += 1
            p0 = wps[self._seg]
            p1 = wps[self._seg + 1]
            dx      = p1[0] - p0[0]
            dy      = p1[1] - p0[1]
            seg_len = math.hypot(dx, dy) or 1e-6

        ratio         = min(self._prog / seg_len, 1.0)
        self._cur_x   = p0[0] + dx * ratio
        self._cur_y   = p0[1] + dy * ratio
        self._cur_yaw = math.atan2(dy, dx)
        return self._cur_x, self._cur_y, self._cur_yaw

    # ── odom rate limit ───────────────────────────────────────────────────────
    def odom_due(self, min_interval_us: int) -> bool:
        now = _now_us()
        if now - self._last_odom_us < min_interval_us:
            return False
        self._last_odom_us = now
        return True

    # ── 영상 ─────────────────────────────────────────────────────────────────
    def next_jpeg(self) -> bytes | None:
        if not self._cap.isOpened():
            return None
        ret, frame = self._cap.read()
        if not ret:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self._cap.read()
            if not ret:
                return None
        ok, buf = cv2.imencode(
            '.jpg', frame,
            [cv2.IMWRITE_JPEG_QUALITY, self._quality],
        )
        return buf.tobytes() if ok else None

    def video_ok(self) -> bool:
        return self._cap.isOpened()

    def release(self) -> None:
        self._cap.release()


# ─── 메인 노드 ────────────────────────────────────────────────────────────────
class SimExecutorNode(Node):

    _KEY_HELP = (
        '  s     시작  (pose 추종 + 영상 송출)\n'
        '  v2    spot_02 탐지 ON\n'
        '  f2    spot_02 탐지 OFF\n'
        '  v3    spot_03 탐지 ON\n'
        '  f3    spot_03 탐지 OFF\n'
        '  q     종료\n'
    )

    def __init__(self):
        super().__init__('sim_executor_node')

        # ── 파라미터 ──────────────────────────────────────────────────────────
        self.declare_parameter('rpi5_ip',          'REQUIRED')
        self.declare_parameter('bridge_port',      BRIDGE_PORT)
        self.declare_parameter('video_path_02',    'REQUIRED')
        self.declare_parameter('video_path_03',    'REQUIRED')
        self.declare_parameter('udp_robot_id_02',  1)   # 0-based: spot_02=1
        self.declare_parameter('udp_robot_id_03',  2)   # 0-based: spot_03=2
        self.declare_parameter('robot_speed_mps',  0.4)
        self.declare_parameter('pose_send_hz',     10.0)
        self.declare_parameter('video_fps',        15)
        self.declare_parameter('jpeg_quality',     80)
        self.declare_parameter('max_size_bytes',   200_000)

        rpi5_ip   = self.get_parameter('rpi5_ip').value
        port      = int(self.get_parameter('bridge_port').value)
        video_02  = self.get_parameter('video_path_02').value
        video_03  = self.get_parameter('video_path_03').value
        udp_id_02 = int(self.get_parameter('udp_robot_id_02').value)
        udp_id_03 = int(self.get_parameter('udp_robot_id_03').value)
        speed     = float(self.get_parameter('robot_speed_mps').value)
        pose_hz   = float(self.get_parameter('pose_send_hz').value)
        video_fps = int(self.get_parameter('video_fps').value)
        quality   = int(self.get_parameter('jpeg_quality').value)
        max_size  = int(self.get_parameter('max_size_bytes').value)

        for label, val in [
            ('rpi5_ip',        rpi5_ip),
            ('video_path_02',  video_02),
            ('video_path_03',  video_03),
        ]:
            if val == 'REQUIRED':
                self.get_logger().fatal(f'파라미터 {label} 미설정')
                raise RuntimeError(f'{label} 파라미터 필수')

        self._odom_min_us = int(1_000_000 / pose_hz)

        # ── RobotSim ──────────────────────────────────────────────────────────
        self._robots: dict[str, RobotSim] = {
            'spot_02': RobotSim('spot_02', udp_id_02, video_02, speed, quality, max_size),
            'spot_03': RobotSim('spot_03', udp_id_03, video_03, speed, quality, max_size),
        }
        for rid, sim in self._robots.items():
            if not sim.video_ok():
                self.get_logger().warn(f'[{rid}] 영상 열기 실패 — 영상 송출 비활성화')

        # ── 실행 상태 ─────────────────────────────────────────────────────────
        self._running = False
        self._t_last  = time.monotonic()

        # ── UDP ───────────────────────────────────────────────────────────────
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._dst  = (rpi5_ip, port)

        # FPS 모니터링
        self._fps_cnt = {rid: 0 for rid in self._robots}
        self._fps_t   = time.monotonic()

        # ── QoS (global_path 수신용만) ────────────────────────────────────────
        transient_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Subscriber ────────────────────────────────────────────────────────
        for rid in self._robots:
            self.create_subscription(
                GlobalPathWaypoints,
                f'/planning/global_path/{rid}',
                lambda msg, r=rid: self._path_cb(msg, r),
                transient_qos,
            )

        # ── 타이머 ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / pose_hz,   self._pose_timer)
        self.create_timer(1.0 / video_fps, self._video_timer)

        # ── 키보드 스레드 ─────────────────────────────────────────────────────
        threading.Thread(
            target=self._keyboard_loop, daemon=True, name='kb',
        ).start()

        self.get_logger().info(
            f'sim_executor_node 시작\n'
            f'  UDP    : {rpi5_ip}:{port}\n'
            f'  spot_02: robot_id={udp_id_02}  {video_02}\n'
            f'  spot_03: robot_id={udp_id_03}  {video_03}\n'
            f'  속도={speed}m/s  pose={pose_hz}Hz  video={video_fps}fps\n'
            f'  ▶ 경로 수신 후 [s] 를 눌러 시작하십시오.'
        )

    # ── 콜백: global_path ─────────────────────────────────────────────────────
    def _path_cb(self, msg: GlobalPathWaypoints, robot_id: str) -> None:
        sim = self._robots.get(robot_id)
        if sim is None:
            return
        sim.set_path(msg.waypoints)
        self.get_logger().info(
            f'[{robot_id}] 경로 수신  waypoints={len(msg.waypoints)}'
        )

    # ── 타이머: pose (PKT_TYPE_ODOM) ─────────────────────────────────────────
    def _pose_timer(self) -> None:
        if not self._running:
            return

        now  = time.monotonic()
        dt   = now - self._t_last
        self._t_last = now

        for rid, sim in self._robots.items():
            x, y, yaw = sim.advance(dt)

            if not sim.odom_due(self._odom_min_us):
                continue

            pkt = _build_odom(sim.udp_id, sim.odom_seq, x, y, yaw)
            sim.odom_seq += 1
            self._send(pkt)

    # ── 타이머: 영상 (PKT_TYPE_IMAGE) ────────────────────────────────────────
    def _video_timer(self) -> None:
        if not self._running:
            return

        for rid, sim in self._robots.items():
            jpeg = sim.next_jpeg()
            if jpeg is None:
                continue
            if len(jpeg) > sim._max_size:
                self.get_logger().warn(f'[{rid}] 프레임 너무 큼: {len(jpeg)}B — skip')
                continue

            for pkt in _build_image_fragments(sim.udp_id, sim.img_seq, jpeg):
                self._send(pkt)
            sim.img_seq = (sim.img_seq + 1) & 0xFFFF_FFFF
            self._fps_cnt[rid] = self._fps_cnt.get(rid, 0) + 1

        now = time.monotonic()
        if now - self._fps_t >= 1.0:
            parts = '  '.join(f'{r}={c}fps' for r, c in self._fps_cnt.items())
            self.get_logger().info(f'[MJPEG] {parts}')
            for r in self._fps_cnt:
                self._fps_cnt[r] = 0
            self._fps_t = now

    # ── person_detected (PKT_TYPE_EVENT) ─────────────────────────────────────
    def _send_victim(self, rid: str, detected: bool) -> None:
        sim = self._robots.get(rid)
        if sim is None:
            return
        pkt = _build_event(sim.udp_id, sim.event_seq, detected)
        sim.event_seq += 1
        self._send(pkt)
        label = 'ON  ✓' if detected else 'OFF ✗'
        self.get_logger().info(f'[EVENT/person_detected/{rid}] {label}')
        print(f'[EVENT/person_detected/{rid}] {label}')

    # ── UDP 송신 ──────────────────────────────────────────────────────────────
    def _send(self, data: bytes) -> None:
        try:
            self._sock.sendto(data, self._dst)
        except OSError as e:
            self.get_logger().error(f'UDP 오류: {e}')

    # ── 키보드 ───────────────────────────────────────────────────────────────
    def _keyboard_loop(self) -> None:
        print(f'[sim_executor] 키 입력 대기:\n{self._KEY_HELP}')
        for line in sys.stdin:
            key = line.strip().lower()

            if key == 's':
                if self._running:
                    print('[sim_executor] 이미 실행 중입니다.')
                else:
                    self._running = True
                    self._t_last  = time.monotonic()
                    print('[sim_executor] ▶ 시작 — pose 추종 + 영상 송출')

            elif key == 'v2':
                self._send_victim('spot_02', True)
            elif key == 'f2':
                self._send_victim('spot_02', False)
            elif key == 'v3':
                self._send_victim('spot_03', True)
            elif key == 'f3':
                self._send_victim('spot_03', False)

            elif key == 'q':
                self.get_logger().info('종료 요청')
                rclpy.shutdown()
                break

            else:
                print(f'[sim_executor] 알 수 없는 입력: "{key}"\n{self._KEY_HELP}')

    # ── 종료 ─────────────────────────────────────────────────────────────────
    def destroy_node(self):
        for sim in self._robots.values():
            sim.release()
        self._sock.close()
        super().destroy_node()


# ─── 진입점 ───────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimExecutorNode()
    except RuntimeError:
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
