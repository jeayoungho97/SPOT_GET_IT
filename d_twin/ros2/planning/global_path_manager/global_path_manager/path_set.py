"""
path_set.py

역할:
  - 기하학적 최소 이격 거리(SAFE_DISTANCE = 1.2) 적용 (END 및 START 포인트 부근 1.5m 배제)
  - 가중치 역전(PROXIMITY > CROSS)을 통한 우측 겹침(Funneling) 현상 완벽 해결
  - 맵 물리적 경계 기준 동적 Clearance(0.3m) 계산 및 커버리지 확장 지원
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Set, Tuple

import numpy as np
import yaml

# ── 탐색 설정 ─────────────────────────────────────────────
MAX_WAYPOINTS      = 6      
SEARCH_GRID_STEP   = 1.0    

# ── 격자 / 반경 설정 ──────────────────────────────────────
GRID_STEP         = 1.0
FOOTPRINT_RADIUS  = 0.3    
SEPARATION_RADIUS = 0.6    
COVER_RADIUS      = 1.0    
COLLINEAR_RADIUS  = 0.1
PATH_SAMPLE_D     = 0.2

# ── 안전 이격 거리 설정 ───────────────────────────────────
SAFE_DISTANCE     = 1.2    

# ── 선정 가중치 ───────────────────────────────────────────
COVERAGE_WEIGHT         = 8.0    
SEPARATION_WEIGHT       = 60.0   
COVERAGE_OVERLAP_WEIGHT = 5.0    
OVERLAP_WEIGHT          = 150.0  
COLLINEAR_WEIGHT        = 200.0  
CROSS_PENALTY           = 5000   # 교차 허용 (물리적 병렬 충돌 방지를 위해 상대적으로 하향)
PROXIMITY_PENALTY       = 15000  # 근접 주행 절대 금지 (선간 거리 강제 확보)
WP_COUNT_PENALTY        = 2.0    
OUTWARD_PUSH_WEIGHT     = 100.0  
FINAL_SEG_PENALTY       = 30.0   

Waypoint = Tuple[float, float]
Cell     = Tuple[int, int]


class MapConfig:
    def __init__(self, path: str):
        with open(path, "r") as f:
            cfg = yaml.safe_load(f)["map"]
        self.starts:      Dict[str, Waypoint] = {k: tuple(v) for k, v in cfg["starts"].items()}
        self.end:         Waypoint             = tuple(cfg["end"])
        self.lobby:       dict                 = cfg["lobby"]
        self.corridor_1:  Optional[dict]       = cfg.get("corridor_1")
        self.wp_sampling: dict                 = cfg["waypoint_sampling"]
        self.obstacles:   List[dict]           = cfg.get("obstacles", [])

    def xs(self) -> List[float]:
        s = self.wp_sampling
        return [round(x, 2) for x in np.arange(s["x_min"], s["x_max"] + 1e-6, s["x_step"])]

    def ys(self) -> List[float]:
        s = self.wp_sampling
        return [round(y, 2) for y in np.arange(s["y_min"], s["y_max"] + 1e-6, s["y_step"])]


# ── 유효성 및 구역 판정 ─────────────────────────

def _is_in_map(x: float, y: float, cfg: MapConfig, use_clearance: bool = True) -> bool:
    l = cfg.lobby
    margin = 0.0 if use_clearance else 0.3
    
    in_lobby = (l["x_min"] - margin <= x <= l["x_max"] + margin and
                l["y_min"] - margin <= y <= l["y_max"] + margin)
    
    in_corr = False
    if cfg.corridor_1:
        c = cfg.corridor_1
        in_corr = (c["x_min"] - margin <= x <= c["x_max"] + margin and
                   c["y_min"] - margin <= y <= c["y_max"] + margin)
        
    return in_lobby or in_corr

def _pt_blocked(x: float, y: float, cfg: MapConfig, use_clearance: bool = True) -> bool:
    if not _is_in_map(x, y, cfg, use_clearance):
        return True
        
    c_margin = 0.3 if use_clearance else 0.0
    if c_margin > 0:
        l = cfg.lobby
        c = cfg.corridor_1
        
        if x < l["x_min"] + c_margin: return True
        if y < l["y_min"] + c_margin: return True
        if x > l["x_max"] - c_margin: return True
        
        if not c:
            if y > l["y_max"] - c_margin: return True
        else:
            if y > l["y_max"] - c_margin:
                if not (c["x_min"] + c_margin <= x <= c["x_max"] - c_margin):
                    return True
            if y >= l["y_max"] - c_margin:
                if x < c["x_min"] + c_margin: return True
                if x > c["x_max"] - c_margin: return True
                if y > c["y_max"] - c_margin: return True

    for obs in cfg.obstacles:
        obs_c = obs.get("clearance", 0.0) if use_clearance else 0.0
        if (obs["x_min"] - obs_c <= x <= obs["x_max"] + obs_c and
                obs["y_min"] - obs_c <= y <= obs["y_max"] + obs_c):
            return True
            
    return False

def _seg_blocked(p1: Waypoint, p2: Waypoint, cfg: MapConfig, n: int = 12) -> bool:
    x1, y1 = p1; x2, y2 = p2
    for i in range(n + 1):
        t = i / n
        if _pt_blocked(x1 + t*(x2-x1), y1 + t*(y2-y1), cfg, use_clearance=True):
            return True
    return False

def _path_valid(wps: List[Waypoint], cfg: MapConfig) -> bool:
    return all(not _seg_blocked(wps[i], wps[i+1], cfg) for i in range(len(wps)-1))


# ── 격자 유틸 ─────────────────────────────────────────────
def _to_cell(x: float, y: float, x_min: float, y_min: float) -> Cell:
    return (math.floor((x - x_min) / GRID_STEP), math.floor((y - y_min) / GRID_STEP))

def _cell_center(cell: Cell, x_min: float, y_min: float) -> Tuple[float, float]:
    return (x_min + (cell[0] + 0.5) * GRID_STEP,
            y_min + (cell[1] + 0.5) * GRID_STEP)

def _build_map_cells(cfg: MapConfig) -> FrozenSet[Cell]:
    cells: Set[Cell] = set()
    x_min, y_min = 0.0, 0.0
    x = 0.0 + GRID_STEP / 2
    while x < 13.0:
        y = 0.0 + GRID_STEP / 2
        while y < 13.0:
            if not _pt_blocked(x, y, cfg, use_clearance=False):
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
        bc = math.floor((px - x_min) / GRID_STEP)
        br = math.floor((py - y_min) / GRID_STEP)
        for dc in range(-r_idx, r_idx + 1):
            for dr in range(-r_idx, r_idx + 1):
                cell = (bc + dc, br + dr)
                if cell not in all_cells:
                    continue
                cx, cy = _cell_center(cell, x_min, y_min)
                if (cx - px)**2 + (cy - py)**2 <= r2:
                    covered.add(cell)
    return frozenset(covered)


# ── 교차 및 선간 거리 판정 ──────────────────────────────────
def _cross(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

def _point_on_segment(p, a, b) -> bool:
    if abs(_cross(a, b, p)) > 1e-5: return False
    if min(a[0], b[0]) - 1e-5 <= p[0] <= max(a[0], b[0]) + 1e-5 and \
       min(a[1], b[1]) - 1e-5 <= p[1] <= max(a[1], b[1]) + 1e-5:
        return True
    return False

def _segments_intersect_strict(a1, a2, b1, b2) -> bool:
    d1 = _cross(b1, b2, a1); d2 = _cross(b1, b2, a2)
    d3 = _cross(a1, a2, b1); d4 = _cross(a1, a2, b2)
    return (((d1 > 1e-5 and d2 < -1e-5) or (d1 < -1e-5 and d2 > 1e-5)) and
            ((d3 > 1e-5 and d4 < -1e-5) or (d3 < -1e-5 and d4 > 1e-5)))

def _paths_cross(wps_a: List[Waypoint], wps_b: List[Waypoint]) -> bool:
    segs_a = [(wps_a[i], wps_a[i+1]) for i in range(len(wps_a)-1)]
    segs_b = [(wps_b[i], wps_b[i+1]) for i in range(len(wps_b)-1)]
    for a1, a2 in segs_a:
        for b1, b2 in segs_b:
            if _segments_intersect_strict(a1, a2, b1, b2): return True
    for wp_a in wps_a[:-1]:
        for b1, b2 in segs_b:
            if _point_on_segment(wp_a, b1, b2): return True
    for wp_b in wps_b[:-1]:
        for a1, a2 in segs_a:
            if _point_on_segment(wp_b, a1, a2): return True
    return False

def _dist_pt_seg(p: Waypoint, a: Waypoint, b: Waypoint) -> float:
    l2 = (a[0]-b[0])**2 + (a[1]-b[1])**2
    if l2 == 0: return math.hypot(p[0]-a[0], p[1]-a[1])
    t = max(0, min(1, ((p[0]-a[0])*(b[0]-a[0]) + (p[1]-a[1])*(b[1]-a[1])) / l2))
    proj = (a[0] + t*(b[0]-a[0]), a[1] + t*(b[1]-a[1]))
    return math.hypot(p[0]-proj[0], p[1]-proj[1])

def _paths_min_dist(
    wps_a: List[Waypoint], wps_b: List[Waypoint],
    end_pt: Waypoint,
    all_starts: Optional[List[Waypoint]] = None,
) -> float:
    min_d = float('inf')

    def is_excluded(p: Waypoint) -> bool:
        if math.hypot(p[0] - end_pt[0], p[1] - end_pt[1]) < 1.5:
            return True
        if all_starts:
            for s in all_starts:
                if math.hypot(p[0] - s[0], p[1] - s[1]) < 1.5:
                    return True
        return False

    for p in wps_a:
        if is_excluded(p): continue
        for i in range(len(wps_b)-1):
            d = _dist_pt_seg(p, wps_b[i], wps_b[i+1])
            if d < min_d: min_d = d

    for p in wps_b:
        if is_excluded(p): continue
        for i in range(len(wps_a)-1):
            d = _dist_pt_seg(p, wps_a[i], wps_a[i+1])
            if d < min_d: min_d = d

    return min_d


# ── 데이터 클래스 ─────────────────────────────────────────
@dataclass
class GlobalPath:
    path_id:    str
    robot_key:  str
    waypoints:  List[Waypoint]
    pivot:      float = 0.0
    footprint:  FrozenSet[Cell] = field(default_factory=frozenset)
    separation: FrozenSet[Cell] = field(default_factory=frozenset)
    coverage:   FrozenSet[Cell] = field(default_factory=frozenset)
    collinear:  FrozenSet[Cell] = field(default_factory=frozenset)
    valid:      bool = True


def shape_of(path: GlobalPath) -> str:
    return 'dynamic'


# ── 로봇별 Path Set 동적 생성 ───────────────────────────────
def _build_paths_for(robot_key: str, start: Waypoint,
                     end: Waypoint, xs: List[float], ys: List[float],
                     cfg: MapConfig) -> List[GlobalPath]:
    paths: List[GlobalPath] = []
    
    grid_xs = [round(x, 2) for x in np.arange(1.0, 12.0, SEARCH_GRID_STEP)]
    grid_ys = [round(y, 2) for y in np.arange(1.0, 8.0, SEARCH_GRID_STEP)]
    
    queue = [([start], 'any')]
    seen_wps = set()

    while queue:
        wps, next_dir = queue.pop(0)
        curr = wps[-1]
        
        if _path_valid([curr, end], cfg):
            final_wps = wps + [end]
            if len(final_wps) <= MAX_WAYPOINTS:
                w_tuple = tuple(final_wps)
                if w_tuple not in seen_wps:
                    seen_wps.add(w_tuple)
                    pid = f"{robot_key}_dyn_{len(paths):04d}"
                    paths.append(GlobalPath(path_id=pid, robot_key=robot_key, waypoints=final_wps, pivot=0.0))
        
        if len(wps) >= MAX_WAYPOINTS - 1:
            continue
            
        if next_dir in ('x', 'any', 'ortho'):
            for nx in grid_xs:
                if nx > curr[0]:
                    nxt = (nx, curr[1])
                    if _path_valid([curr, nxt], cfg):
                        queue.append((wps + [nxt], 'y'))
                        
        if next_dir in ('y', 'any', 'ortho'):
            for ny in grid_ys:
                if ny > curr[1]:
                    nxt = (curr[0], ny)
                    if _path_valid([curr, nxt], cfg):
                        queue.append((wps + [nxt], 'x'))
                        
        if next_dir in ('diag', 'any'):
            for d in np.arange(SEARCH_GRID_STEP, 10.0, SEARCH_GRID_STEP):
                nx = round(curr[0] + d, 2)
                ny = round(curr[1] + d, 2)
                if nx <= 12.0 and ny <= 8.0:
                    nxt = (nx, ny)
                    if _path_valid([curr, nxt], cfg):
                        queue.append((wps + [nxt], 'ortho'))
                        
    return paths


def build_path_set(cfg: MapConfig) -> Dict[str, List[GlobalPath]]:
    xs = cfg.xs(); ys = cfg.ys()
    return {k: _build_paths_for(k, start, cfg.end, xs, ys, cfg)
            for k, start in cfg.starts.items()}


# ── 유효성 + 셀 계산 ──────────────────────────────────────
def validate_path_set(
    path_sets: Dict[str, List[GlobalPath]],
    cfg: MapConfig,
) -> Dict[str, List[GlobalPath]]:
    x_min, y_min = 0.0, 0.0
    all_cells = _build_map_cells(cfg)

    for paths in path_sets.values():
        for path in paths:
            path.valid = _path_valid(path.waypoints, cfg)
            if path.valid:
                path.footprint  = _cells_within_radius(path.waypoints, FOOTPRINT_RADIUS,  all_cells, x_min, y_min)
                path.separation = _cells_within_radius(path.waypoints, SEPARATION_RADIUS, all_cells, x_min, y_min)
                path.coverage   = _cells_within_radius(path.waypoints, COVER_RADIUS,      all_cells, x_min, y_min)
                path.collinear  = _cells_within_radius(path.waypoints, COLLINEAR_RADIUS,  all_cells, x_min, y_min)
            else:
                path.footprint = path.separation = path.coverage = path.collinear = frozenset()
    return path_sets


# ── 각도 기반 shape 배정 ──────────────────────────────────
def _angle_to_shape(ratio: float) -> str:
    if ratio < 0.33: return 'x_first'
    elif ratio < 0.66: return 'diagonal'
    else: return 'y_first'


# ── 공간 분리 가중치 산출 ──────────────────
def get_spatial_bonus(wps: List[Waypoint], target_shape: str) -> float:
    scores = []
    for wp in wps:
        nx = wp[0] / 12.0
        ny = wp[1] / 8.0
        
        if target_shape == 'y_first':
            scores.append(ny + (1.0 - nx))
        elif target_shape == 'x_first':
            scores.append(nx + (1.0 - ny))
        else:
            scores.append(- (abs(nx - 0.5) + abs(ny - 0.5)))
            
    return max(scores) * OUTWARD_PUSH_WEIGHT


# ── 경로 선정 ─────────────────────────────
def select_paths(
    path_sets: Dict[str, List[GlobalPath]],
    starts:    Optional[Dict[str, Waypoint]] = None,
) -> Dict[str, GlobalPath]:
    
    first_path = next((p[0] for p in path_sets.values() if p), None)
    end_pt = first_path.waypoints[-1] if first_path else (11.2, 7.0)

    if starts is None:
        starts = {k: paths[0].waypoints[0] for k, paths in path_sets.items() if paths}

    n = len(starts)
    sorted_robots = sorted(starts.keys(), key=lambda k: starts[k][1] - starts[k][0], reverse=True)
    target_shapes = {rk: _angle_to_shape(i / (n - 1) if n > 1 else 0.5) for i, rk in enumerate(reversed(sorted_robots))}

    eval_order = [rk for rk in sorted_robots if target_shapes[rk] != 'diagonal'] + \
                 [rk for rk in sorted_robots if target_shapes[rk] == 'diagonal']

    selected:          Dict[str, GlobalPath] = {}
    covered_cov:       Set[Cell] = set()
    covered_sep:       Set[Cell] = set()
    covered_foot:      Set[Cell] = set()
    covered_collinear: Set[Cell] = set()
    
    start_pts = list(starts.values())

    for robot_key in eval_order:
        candidates = [p for p in path_sets[robot_key] if p.valid]
        if not candidates: continue

        target_shape = target_shapes[robot_key]

        for p in candidates:
            p.pivot = get_spatial_bonus(p.waypoints, target_shape)

        def score(p: GlobalPath) -> float:
            cross_p = sum(CROSS_PENALTY for sel in selected.values() if _paths_cross(p.waypoints, sel.waypoints))
            
            prox_p = 0
            for sel in selected.values():
                if _paths_min_dist(p.waypoints, sel.waypoints, end_pt, start_pts) < SAFE_DISTANCE:
                    prox_p += PROXIMITY_PENALTY

            sep_overlap = len(p.separation & covered_sep)
            
            final_seg_len = math.hypot(p.waypoints[-2][0] - p.waypoints[-1][0], p.waypoints[-2][1] - p.waypoints[-1][1])
            final_seg_penalty = final_seg_len * FINAL_SEG_PENALTY

            collinear_ratio = len(p.collinear & covered_collinear) / max(1, len(p.collinear))
            wp_penalty = WP_COUNT_PENALTY * len(p.waypoints)

            return (
                COVERAGE_WEIGHT * len(p.coverage - covered_cov)
                - COVERAGE_OVERLAP_WEIGHT * len(p.coverage & covered_cov)
                - SEPARATION_WEIGHT * sep_overlap
                - OVERLAP_WEIGHT * len(p.footprint & covered_foot)
                + p.pivot
                - cross_p
                - prox_p
                - final_seg_penalty
                - COLLINEAR_WEIGHT * collinear_ratio
                - wp_penalty
            )

        best = max(candidates, key=score)
        selected[robot_key] = best
        
        covered_cov       |= best.coverage
        covered_sep       |= best.separation
        covered_foot      |= best.footprint
        covered_collinear |= best.collinear

    return selected
