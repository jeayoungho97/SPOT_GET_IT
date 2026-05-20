#!/usr/bin/env python3
"""
sim_executor_script.py
======================
프로젝트 : SPOT Get IT
작성일   : 2026-05-18

ros2 launch 없이 python3로 직접 실행하는 독립 스크립트.
stdin이 정상 동작하므로 키보드 입력 가능.

실행
  source ~/robot_ws/install/setup.bash
  python3 sim_executor_script.py

흐름
  1. /planning/global_path/spot_02|03 수신 대기
  2. 수신 즉시 해당 로봇 워커 스레드 자동 시작
     → Pose   UDP (PKT_TYPE_ODOM)
     → MJPEG  UDP (PKT_TYPE_IMAGE)
  3. 두 로봇 모두 시작된 이후 키보드로 person_detected 전송
     v2 / f2 / v3 / f3

키보드 (Enter 후 처리)
  v2   spot_02 탐지 ON
  f2   spot_02 탐지 OFF
  v3   spot_03 탐지 ON
  f3   spot_03 탐지 OFF
  q    종료
"""

import math
import select
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

# ─── 설정 (필요 시 수정) ──────────────────────────────────────────────────────
RPI5_IP        = '192.168.0.12'
BRIDGE_PORT    = 9000
# Fleet configuration: one real robot and dummy senders
REAL_ROBOT = 'spot_01'                 # real robot (not simulated here)
DUMMY_ROBOTS = ['spot_02', 'spot_03', 'spot_04']

# Video paths per dummy robot (fallback)
VIDEO_PATH_DEFAULT = '/home/ubuntu/media/spot_02.mp4'
VIDEO_PATHS = {
    'spot_02': '/home/ubuntu/media/spot_02.mp4',
    'spot_03': '/home/ubuntu/media/spot_03.mp4',
    'spot_04': '/home/ubuntu/media/spot_04.mp4',
}
ROBOT_SPEED    = 0.1    # m/s
POSE_HZ        = 10.0
VIDEO_FPS      = 15
JPEG_QUALITY   = 80
MAX_FRAME_SIZE = 200_000

# ─── proto.h 상수 ─────────────────────────────────────────────────────────────
PKT_TYPE_IMAGE = 0x01
PKT_TYPE_ODOM  = 0x03
PKT_TYPE_EVENT = 0x07
PKT_TYPE_PATH_PROGRESS = 0x0C
PROTO_MTU      = 1400

EVENT_SEVERITY_CRITICAL    = 4
EVENT_TYPE_VICTIM_DETECTED = 8

_HDR_FMT   = '<BBHHHIIQ'
_ODOM_FMT  = '<fffffff'
_EVENT_FMT = '<BBHi96s'
_HDR_SIZE   = struct.calcsize(_HDR_FMT)
_ODOM_SIZE  = struct.calcsize(_ODOM_FMT)
_EVENT_SIZE = struct.calcsize(_EVENT_FMT)

# PathProgress payload format (matches PathProgressPayload in proto.h)
_PATHPROG_FMT  = '<16sQBBBBIIfIfffffffff'
_PATHPROG_SIZE = struct.calcsize(_PATHPROG_FMT)


# robot_id (0-based) -> UDP id mapping
# Reserve 0 for the real robot, simulated dummies start from 1
UDP_ROBOT_ID = {REAL_ROBOT: 0}
for i, rid in enumerate(DUMMY_ROBOTS):
    UDP_ROBOT_ID[rid] = i + 1


def _now_us() -> int:
    return int(time.monotonic() * 1_000_000)


def _hdr(pkt_type, robot_id, payload_len, frame_id,
         frag_idx=0, frag_total=1, offset=0) -> bytes:
    return struct.pack(_HDR_FMT, pkt_type, robot_id,
                       frag_idx, frag_total, payload_len,
                       frame_id, offset, _now_us())


def _build_odom(robot_id, seq, x, y, theta) -> bytes:
    return (_hdr(PKT_TYPE_ODOM, robot_id, _ODOM_SIZE, seq)
            + struct.pack(_ODOM_FMT, x, y, theta, 0.0, 0.0, 0.0, 0.0))


def _build_event(robot_id, seq, detected) -> bytes:
    msg_b = b'person detected' if detected else b'person cleared'
    return (_hdr(PKT_TYPE_EVENT, robot_id, _EVENT_SIZE, seq)
            + struct.pack(_EVENT_FMT,
                          EVENT_SEVERITY_CRITICAL, 0,
                          EVENT_TYPE_VICTIM_DETECTED,
                          1 if detected else 0,
                          msg_b))


def _build_image_frags(robot_id, seq, jpeg) -> list:
    frag_total = math.ceil(len(jpeg) / PROTO_MTU)
    pkts = []
    for idx in range(frag_total):
        offset = idx * PROTO_MTU
        chunk  = jpeg[offset: offset + PROTO_MTU]
        pkts.append(_hdr(PKT_TYPE_IMAGE, robot_id, len(chunk),
                         seq, idx, frag_total, offset) + chunk)
    return pkts


# ─── 로봇 워커 ────────────────────────────────────────────────────────────────
class RobotWorker:

    def __init__(self, ros_id: str, sock: socket.socket, dst: tuple):
        self.ros_id  = ros_id
        self.udp_id  = UDP_ROBOT_ID[ros_id]
        self._sock   = sock
        self._dst    = dst

        # 경로
        self._wps: list[tuple[float, float]] = []
        self._seg    = 0
        self._prog   = 0.0
        self._cur_x  = 0.0
        self._cur_y  = 0.0
        self._cur_yaw = 0.0
        self._wps_lock = threading.Lock()

        # 시퀀스
        self._odom_seq  = 0
        self._event_seq = 0
        self._img_seq   = 0
        self._last_odom = 0
        self._progress_seq = 0

        # 영상
        video_path = VIDEO_PATHS.get(ros_id, VIDEO_PATH_DEFAULT)
        self._cap = cv2.VideoCapture(video_path)
        if not self._cap.isOpened():
            print(f'[{ros_id}] ⚠ 영상 열기 실패')

        self._stop_evt = threading.Event()
        self._running  = False
        self._fps_cnt  = 0
        self._fps_t    = time.monotonic()
        self._pose_thread = None
        self._video_thread = None

    def set_path(self, waypoints_msg: list) -> None:
        with self._wps_lock:
            self._wps  = [(wp.x_m, wp.y_m) for wp in waypoints_msg]
            self._seg  = 0
            self._prog = 0.0
            if self._wps:
                self._cur_x, self._cur_y = self._wps[0]
                if len(self._wps) > 1:
                    next_x, next_y = self._wps[1]
                    self._cur_yaw = math.atan2(next_y - self._cur_y, next_x - self._cur_x)
        print(f'[{self.ros_id}] 경로 수신 waypoints={len(self._wps)}')

    def send_path_progress(self) -> None:
        with self._wps_lock:
            wps = list(self._wps)
            seg = self._seg
            prog = self._prog
            cur_x = self._cur_x
            cur_y = self._cur_y

        n = len(wps)
        path_ok = 1 if n > 0 else 0
        pose_ok = 1
        goal_reached = 0
        waypoint_idx = 0
        total_waypoints = n
        mission_progress = 0.0
        nearest_index = 0
        distance_to_nearest_m = 0.0
        nearest_x = 0.0
        nearest_y = 0.0
        target_x = 0.0
        target_y = 0.0
        target_heading = 0.0
        heading_error = 0.0
        distance_to_target = 0.0
        distance_to_goal = 0.0

        if n > 0:
            best_d = float('inf')
            for i, (wx, wy) in enumerate(wps):
                d = math.hypot(cur_x - wx, cur_y - wy)
                if d < best_d:
                    best_d = d
                    nearest_index = i
                    nearest_x = wx
                    nearest_y = wy
            distance_to_nearest_m = float(best_d)

            tgt = min(seg + 1, n - 1)
            target_x, target_y = wps[tgt]
            waypoint_idx = tgt

            if n > 1:
                total_path_len = 0.0
                for i in range(n - 1):
                    a = wps[i]; b = wps[i + 1]
                    total_path_len += math.hypot(b[0] - a[0], b[1] - a[1])
                traveled = 0.0
                for i in range(seg):
                    a = wps[i]; b = wps[i + 1]
                    traveled += math.hypot(b[0] - a[0], b[1] - a[1])
                if seg < n - 1:
                    a = wps[seg]; b = wps[seg + 1]
                    seg_len = math.hypot(b[0] - a[0], b[1] - a[1]) or 1e-6
                    traveled += min(prog, seg_len)
                mission_progress = float(min(max(traveled / (total_path_len or 1.0), 0.0), 1.0))

            distance_to_target = float(math.hypot(cur_x - target_x, cur_y - target_y))
            distance_to_goal   = float(math.hypot(cur_x - wps[-1][0], cur_y - wps[-1][1]))
            goal_reached = 1 if (seg >= n - 1 and distance_to_goal < 0.5) else 0

        ts = _now_us()
        payload = struct.pack(
            _PATHPROG_FMT,
            self.ros_id.encode('utf-8')[:16].ljust(16, b'\x00'),
            ts,
            path_ok, pose_ok, goal_reached, 0,
            waypoint_idx, total_waypoints, mission_progress,
            nearest_index, distance_to_nearest_m,
            nearest_x, nearest_y,
            target_x, target_y, target_heading, heading_error,
            distance_to_target, distance_to_goal,
        )
        hdr = _hdr(PKT_TYPE_PATH_PROGRESS, self.udp_id, len(payload), self._progress_seq)
        self._send(hdr + payload)
        self._progress_seq = (self._progress_seq + 1) & 0xFFFFFFFF

    def send_event(self, detected: bool) -> None:
        self._send(_build_event(self.udp_id, self._event_seq, detected))
        self._event_seq += 1
        label = 'ON  ✓' if detected else 'OFF ✗'
        print(f'[EVENT/{self.ros_id}] {label}')

    def stop(self) -> None:
        self._stop_evt.set()
        # join threads if running
        if self._pose_thread and self._pose_thread.is_alive():
            self._pose_thread.join(timeout=1.0)
        if self._video_thread and self._video_thread.is_alive():
            self._video_thread.join(timeout=1.0)
        self._cap.release()
        self._running = False

    def start(self) -> None:
        if self._running:
            return
        self._stop_evt.clear()
        self._t_last = time.monotonic()
        self._pose_thread = threading.Thread(target=self._pose_loop, name=f'pose-{self.ros_id}', daemon=True)
        self._video_thread = threading.Thread(target=self._video_loop, name=f'video-{self.ros_id}', daemon=True)
        self._pose_thread.start()
        self._video_thread.start()
        # ensure immediate odom/path progress is sent when starting
        self._last_odom = 0
        self._running = True
        try:
            self._send(_build_odom(self.udp_id, self._odom_seq, self._cur_x, self._cur_y, self._cur_yaw))
            self._odom_seq += 1
        except Exception:
            pass
        try:
            self.send_path_progress()
        except Exception:
            pass
        print(f'[{self.ros_id}] 자동 송신 시작')

    # ── Pose 스레드 ───────────────────────────────────────────────────────────
    def _pose_loop(self) -> None:
        pose_dt      = 1.0 / POSE_HZ
        odom_min_us  = int(1_000_000 / POSE_HZ)

        while not self._stop_evt.is_set():
            t0  = time.monotonic()
            dt  = t0 - self._t_last
            self._t_last = t0

            with self._wps_lock:
                x, y, yaw = self._advance(dt)

            now_us = _now_us()
            if now_us - self._last_odom >= odom_min_us:
                self._last_odom = now_us
                self._send(_build_odom(self.udp_id, self._odom_seq, x, y, yaw))
                self._odom_seq += 1
                try:
                    self.send_path_progress()
                except Exception:
                    pass

            time.sleep(max(0.0, pose_dt - (time.monotonic() - t0)))

    def _advance(self, dt: float) -> tuple:
        wps = self._wps
        n   = len(wps)
        if n < 2:
            return self._cur_x, self._cur_y, self._cur_yaw
        if self._seg >= n - 1:
            self._cur_x, self._cur_y = wps[-1]
            return self._cur_x, self._cur_y, self._cur_yaw

        p0 = wps[self._seg];  p1 = wps[self._seg + 1]
        dx = p1[0] - p0[0];   dy = p1[1] - p0[1]
        seg_len = math.hypot(dx, dy) or 1e-6

        self._prog += ROBOT_SPEED * dt
        while self._prog >= seg_len and self._seg < n - 2:
            self._prog -= seg_len
            self._seg  += 1
            p0 = wps[self._seg];  p1 = wps[self._seg + 1]
            dx = p1[0] - p0[0];   dy = p1[1] - p0[1]
            seg_len = math.hypot(dx, dy) or 1e-6

        r             = min(self._prog / seg_len, 1.0)
        self._cur_x   = p0[0] + dx * r
        self._cur_y   = p0[1] + dy * r
        self._cur_yaw = math.atan2(dy, dx)
        return self._cur_x, self._cur_y, self._cur_yaw

    # ── MJPEG 스레드 ──────────────────────────────────────────────────────────
    def _video_loop(self) -> None:
        video_dt = 1.0 / VIDEO_FPS
        next_send = time.monotonic()

        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            frame = None

            if self._cap.isOpened():
                now = t0
                if now >= next_send:
                    while now >= next_send:
                        grabbed = self._cap.grab()
                        if not grabbed:
                            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                            break
                        next_send += video_dt
                        now = time.monotonic()
                    ret, frame = self._cap.retrieve()
                    if not ret:
                        self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        ret, frame = self._cap.read()
                if frame is not None:
                    ok, buf = cv2.imencode(
                        '.jpg', frame,
                        [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
                    )
                    if ok:
                        jpeg = buf.tobytes()
                        if len(jpeg) <= MAX_FRAME_SIZE:
                            for pkt in _build_image_frags(
                                self.udp_id, self._img_seq, jpeg
                            ):
                                self._send(pkt)
                            self._img_seq = (self._img_seq + 1) & 0xFFFF_FFFF
                            self._fps_cnt += 1

            now = time.monotonic()
            if now - self._fps_t >= 1.0:
                print(f'[MJPEG/{self.ros_id}] {self._fps_cnt}fps')
                self._fps_cnt = 0
                self._fps_t   = now

            time.sleep(max(0.0, video_dt - (time.monotonic() - t0)))

    def _send(self, data: bytes) -> None:
        try:
            self._sock.sendto(data, self._dst)
        except OSError as e:
            print(f'[{self.ros_id}] UDP 오류: {e}')


# ─── ROS2 구독 노드 ───────────────────────────────────────────────────────────
class PathSubscriber(Node):

    def __init__(self, workers: dict):
        super().__init__('sim_executor_script')
        self._workers = workers

        transient_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        for rid in workers:
            self.create_subscription(
                GlobalPathWaypoints,
                f'/planning/global_path/{rid}',
                lambda msg, r=rid: self._path_cb(msg, r),
                transient_qos,
            )

    def _path_cb(self, msg: GlobalPathWaypoints, robot_id: str) -> None:
        w = self._workers.get(robot_id)
        if w:
            w.set_path(msg.waypoints)
            try:
                w.send_path_progress()
            except Exception:
                pass




# ─── 메인 ─────────────────────────────────────────────────────────────────────
_KEY_HELP = (
    '  s0   spot_02 + spot_03 동시 시작\n'
    '  s2   spot_02 시작\n'
    '  s3   spot_03 시작\n'
    '  s4   spot_04 시작\n'
    '  v2   spot_02 탐지 ON\n'
    '  f2   spot_02 탐지 OFF\n'
    '  v3   spot_03 탐지 ON\n'
    '  f3   spot_03 탐지 OFF\n'
    '  v4   spot_04 탐지 ON\n'
    '  f4   spot_04 탐지 OFF\n'
    '  q    종료\n'
)


def main() -> None:
    rclpy.init()

    sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst     = (RPI5_IP, BRIDGE_PORT)
    workers = {}
    # create workers only for dummy senders
    for rid in DUMMY_ROBOTS:
        workers[rid] = RobotWorker(rid, sock, dst)

    node     = PathSubscriber(workers)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    print(f'[sim_executor_script] 시작')
    print(f'  UDP → {RPI5_IP}:{BRIDGE_PORT}')
    print(f'  /planning/global_path/spot_02|03 수신 대기 중...')
    print(f'  경로 수신 후 키 입력으로 시작하십시오.')
    print(f'키보드:\n{_KEY_HELP}')

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)

            r, _, _ = select.select([sys.stdin], [], [], 0)
            if not r:
                continue

            line = sys.stdin.readline()
            if not line:
                break
            key = line.strip().lower()

            if   key == 's0':
                workers['spot_02'].start()
                workers['spot_03'].start()
                # start spot_04 as part of s0 as requested
                if 'spot_04' in workers:
                    workers['spot_04'].start()
            elif key == 's4': workers['spot_04'].start()
            elif key == 's2': workers['spot_02'].start()
            elif key == 's3': workers['spot_03'].start()
            elif key == 'v2': workers['spot_02'].send_event(True)
            elif key == 'f2': workers['spot_02'].send_event(False)
            elif key == 'v3': workers['spot_03'].send_event(True)
            elif key == 'f3': workers['spot_03'].send_event(False)
            elif key == 'v4': workers['spot_04'].send_event(True)
            elif key == 'f4': workers['spot_04'].send_event(False)
            elif key == 'q':  break
            else: print(f'알 수 없는 입력: "{key}"\n{_KEY_HELP}')

    except KeyboardInterrupt:
        pass
    finally:
        for w in workers.values():
            w.stop()
        sock.close()
        node.destroy_node()
        rclpy.shutdown()
        print('[sim_executor_script] 종료.')


if __name__ == '__main__':
    main()
