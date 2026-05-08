"""
path_set.py

역할:
  - map.yaml 로드 (MapConfig)
  - 로봇별 출발지 기준 path set 생성 (build_path_set)
  - 장애물 기준 유효성 + footprint/coverage 셀 계산 (validate_path_set)
  - 경로 선정 (select_paths)
    현재: _manual_select — 원하는 waypoints를 직접 지정
    추후: 알고리즘 기반 자동 선정으로 교체 예정

좌표계: 원점 좌측 하단 / +X 문 벽 방향 / +Y 단상 방향 / 단위 m
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Set, Tuple

import numpy as np
import yaml

# ── 격자 / 반경 설정 ──────────────────────────────────────
GRID_STEP        = 0.5    # 커버리지 격자 셀 크기 (m)
FOOTPRINT_RADIUS = 0.3    # 경로 발자국 반경 (m) — 겹침 판정용
COVER_RADIUS     = 1.0    # 탐지 커버 반경 (m) — 커버리지 판정용
PATH_SAMPLE_D    = 0.2    # 선분 샘플링 간격 (m)
OVERLAP_WEIGHT   = 3.0    # 겹침 패널티 가중치

Waypoint = Tuple[float, float]
Cell     = Tuple[int, int]


# ── 맵 설정 ───────────────────────────────────────────────
class MapConfig:
    def __init__(self, path: str):
        with open(path, "r") as f:
            cfg = yaml.safe_load(f)["map"]
        self.starts:      Dict[str, Waypoint] = {k: tuple(v) for k, v in cfg["starts"].items()}
        self.end:         Waypoint             = tuple(cfg["end"])
        self.lobby:       dict                 = cfg["lobby"]
        self.wp_sampling: dict                 = cfg["waypoint_sampling"]
        self.obstacles:   List[dict]           = cfg.get("obstacles", [])

    def xs(self) -> List[float]:
        s = self.wp_sampling
        return [round(x, 2) for x in np.arange(s["x_min"], s["x_max"] + 1e-6, s["x_step"])]

    def ys(self) -> List[float]:
        s = self.wp_sampling
        return [round(y, 2) for y in np.arange(s["y_min"], s["y_max"] + 1e-6, s["y_step"])]


# ── 장애물 판정 ───────────────────────────────────────────
def _pt_blocked(x: float, y: float, obstacles: List[dict]) -> bool:
    for obs in obstacles:
        c = obs.get("clearance", 0.0)
        if (obs["x_min"] - c <= x <= obs["x_max"] + c and
                obs["y_min"] - c <= y <= obs["y_max"] + c):
            return True
    return False


def _seg_blocked(p1: Waypoint, p2: Waypoint, obstacles: List[dict], n: int = 12) -> bool:
    x1, y1 = p1; x2, y2 = p2
    for i in range(n + 1):
        t = i / n
        if _pt_blocked(x1 + t*(x2-x1), y1 + t*(y2-y1), obstacles):
            return True
    return False


def _path_valid(wps: List[Waypoint], obstacles: List[dict]) -> bool:
    return all(not _seg_blocked(wps[i], wps[i+1], obstacles) for i in range(len(wps)-1))


# ── 격자 유틸 ─────────────────────────────────────────────
def _to_cell(x: float, y: float, x_min: float, y_min: float) -> Cell:
    return (int((x - x_min) / GRID_STEP), int((y - y_min) / GRID_STEP))


def _cell_center(cell: Cell, x_min: float, y_min: float) -> Tuple[float, float]:
    return (x_min + (cell[0] + 0.5) * GRID_STEP,
            y_min + (cell[1] + 0.5) * GRID_STEP)


def _build_lobby_cells(lobby: dict, obstacles: List[dict]) -> FrozenSet[Cell]:
    cells: Set[Cell] = set()
    x_min, x_max = lobby["x_min"], lobby["x_max"]
    y_min, y_max = lobby["y_min"], lobby["y_max"]
    x = x_min + GRID_STEP / 2
    while x < x_max:
        y = y_min + GRID_STEP / 2
        while y < y_max:
            if not _pt_blocked(x, y, obstacles):
                cells.add(_to_cell(x, y, x_min, y_min))
            y += GRID_STEP
        x += GRID_STEP
    return frozenset(cells)


def _cells_within_radius(waypoints: List[Waypoint], radius: float,
                          all_cells: FrozenSet[Cell],
                          x_min: float, y_min: float) -> FrozenSet[Cell]:
    covered: Set[Cell] = set()
    r2 = radius ** 2
    r_idx = int(radius / GRID_STEP) + 1
    pts = []
    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]; x2, y2 = waypoints[i+1]
        length = math.hypot(x2-x1, y2-y1)
        steps  = max(1, int(length / PATH_SAMPLE_D))
        for s in range(steps + 1):
            t = s / steps
            pts.append((x1 + t*(x2-x1), y1 + t*(y2-y1)))
    for (px, py) in pts:
        bc = int((px - x_min) / GRID_STEP)
        br = int((py - y_min) / GRID_STEP)
        for dc in range(-r_idx, r_idx + 1):
            for dr in range(-r_idx, r_idx + 1):
                cell = (bc + dc, br + dr)
                if cell not in all_cells:
                    continue
                cx, cy = _cell_center(cell, x_min, y_min)
                if (cx - px)**2 + (cy - py)**2 <= r2:
                    covered.add(cell)
    return frozenset(covered)


# ── 데이터 클래스 ─────────────────────────────────────────
@dataclass
class GlobalPath:
    path_id:   str
    robot_key: str
    waypoints: List[Waypoint]        # 출발 포함, 도착 포함, 경유지 수 제한 없음
    footprint: FrozenSet[Cell] = field(default_factory=frozenset)
    coverage:  FrozenSet[Cell] = field(default_factory=frozenset)
    valid:     bool = True


# ── 로봇별 Path Set 생성 ──────────────────────────────────
def _build_paths_for(robot_key: str, start: Waypoint,
                     end: Waypoint, xs: List[float], ys: List[float]) -> List[GlobalPath]:
    """
    한 로봇의 출발지 기준 path set 생성.
    경유지 수에 제한 없음 — 장애물 우회 등 다양한 형태 포함.
    """
    paths: List[GlobalPath] = []
    sx, sy = start; ex, ey = end

    # Y_FIRST: (start) → (sx, pivot_y) → (pivot_x, pivot_y) → (end)
    for idx, (pivot_y, pivot_x) in enumerate(itertools.product(ys, xs)):
        if pivot_x > ex + 0.5 or pivot_y > ey + 0.5:
            continue
        wps = [(sx, sy), (sx, float(pivot_y)), (float(pivot_x), float(pivot_y)), (ex, ey)]
        paths.append(GlobalPath(path_id=f"{robot_key}_yf_{idx:04d}",
                                robot_key=robot_key, waypoints=wps))

    # DIAGONAL 경유지 1개
    dg_xs = [round(x, 2) for x in np.arange(2.5, 11.0, 1.0)]
    dg_ys = [round(y, 2) for y in np.arange(1.0,  7.5, 1.0)]
    for idx, (vx, vy) in enumerate(itertools.product(dg_xs, dg_ys)):
        wps = [(sx, sy), (float(vx), float(vy)), (ex, ey)]
        paths.append(GlobalPath(path_id=f"{robot_key}_dg1_{idx:03d}",
                                robot_key=robot_key, waypoints=wps))

    # DIAGONAL 경유지 2개
    front = list(itertools.product(
        [round(x, 2) for x in np.arange(3.0, 7.0, 1.0)],
        [round(y, 2) for y in np.arange(1.0, 4.5, 1.0)],
    ))
    back = list(itertools.product(
        [round(x, 2) for x in np.arange(7.0, 11.5, 1.0)],
        [round(y, 2) for y in np.arange(3.5,  7.5, 1.0)],
    ))
    for idx, ((fx, fy), (bx, by)) in enumerate(itertools.product(front, back)):
        if fy >= by:
            continue
        wps = [(sx, sy), (float(fx), float(fy)), (float(bx), float(by)), (ex, ey)]
        paths.append(GlobalPath(path_id=f"{robot_key}_dg2_{idx:04d}",
                                robot_key=robot_key, waypoints=wps))

    # X_FIRST: (start) → (pivot_x, sy) → (pivot_x, mid_y) → (end)
    # 출발지에서 최소 1.0m 이상 X 이동해야 의미 있음
    for idx, (pivot_x, mid_y) in enumerate(itertools.product(xs, ys)):
        if pivot_x > ex + 0.5 or mid_y > ey + 0.5:
            continue
        if pivot_x < sx + 1.0:
            continue
        wps = [(sx, sy), (float(pivot_x), sy), (float(pivot_x), float(mid_y)), (ex, ey)]
        paths.append(GlobalPath(path_id=f"{robot_key}_xf_{idx:04d}",
                                robot_key=robot_key, waypoints=wps))

    return paths


def build_path_set(cfg: MapConfig) -> Dict[str, List[GlobalPath]]:
    """로봇별 path set 생성. 반환: {robot_key: [GlobalPath]}"""
    xs = cfg.xs(); ys = cfg.ys()
    return {k: _build_paths_for(k, start, cfg.end, xs, ys)
            for k, start in cfg.starts.items()}


# ── 유효성 + 커버리지 갱신 ────────────────────────────────
def validate_path_set(
    path_sets: Dict[str, List[GlobalPath]],
    cfg: MapConfig,
) -> Dict[str, List[GlobalPath]]:
    """장애물 기준 valid 갱신 + footprint/coverage 셀 계산."""
    obstacles = cfg.obstacles
    lobby     = cfg.lobby
    x_min     = lobby["x_min"]
    y_min     = lobby["y_min"]
    all_cells = _build_lobby_cells(lobby, obstacles)

    for paths in path_sets.values():
        for path in paths:
            path.valid = _path_valid(path.waypoints, obstacles)
            if path.valid:
                path.footprint = _cells_within_radius(
                    path.waypoints, FOOTPRINT_RADIUS, all_cells, x_min, y_min)
                path.coverage  = _cells_within_radius(
                    path.waypoints, COVER_RADIUS, all_cells, x_min, y_min)
            else:
                path.footprint = frozenset()
                path.coverage  = frozenset()
    return path_sets


# ── 경로 선정 ─────────────────────────────────────────────
def select_paths(
    path_sets: Dict[str, List[GlobalPath]],
    starts:    Optional[Dict[str, Waypoint]] = None,
) -> Dict[str, GlobalPath]:
    """
    경로 선정 함수.

    TODO: 알고리즘 기반 자동 선정으로 교체 예정.
    현재: _manual_select — 원하는 waypoints 직접 지정.
    반환: {robot_key: GlobalPath}
    """
    return _manual_select(path_sets)


def _manual_select(path_sets: Dict[str, List[GlobalPath]]) -> Dict[str, GlobalPath]:
    """
    수동 경로 지정.
    원하는 waypoints를 GlobalPath로 직접 생성.

    robot_01 → diagonal : 출발 → 중앙 대각 → 도착
    sim_02   → y_first  : 출발 → 위로 올라간 뒤 → 도착
    sim_03   → x_first  : 출발 → 오른쪽 끝까지 → 도착

    TODO: 알고리즘으로 교체 시 이 함수 대체.
    """
    targets: Dict[str, List[Waypoint]] = {
        "robot_01": [(2.0, 1.0), (6.5, 4.0), (11.2, 7.0)],
        "sim_02":   [(2.0, 2.0), (2.0, 6.5), (11.2, 7.0)],
        "sim_03":   [(3.0, 1.0), (11.0, 1.0), (11.2, 7.0)],
    }

    return {
        robot_key: GlobalPath(
            path_id=f"{robot_key}_manual",
            robot_key=robot_key,
            waypoints=wps,
        )
        for robot_key, wps in targets.items()
        if robot_key in path_sets
    }
