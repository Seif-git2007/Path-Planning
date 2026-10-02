from __future__ import annotations

import math
from typing import List, Optional, Tuple

from src.models import CarPose, Cone, Path2D

Point = Tuple[float, float]

HALF_TRACK_WIDTH = 1.0
STEP = 0.25
MIN_PATH_LENGTH = 7.0
MAX_PATH_LENGTH = 10.0
EXTRA_LENGTH = 1.5
MAX_ARC_ANGLE = math.pi / 2
MAX_CIRCLE_RADIUS = 50.0
ENTRY_DISTANCE = 1.0
SMOOTH_SPACING = 1.0
SMOOTH_ITERATIONS = 3
MIN_TURN_RADIUS = 1.0
LOOKAHEAD = 1.0
SIM_STEP = 0.05


class PathPlanning:
    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        blue, yellow = self._cones_local()

        result = None
        if len(blue) >= 3 or len(yellow) >= 3:
            result = self._curve_from_three_cones(blue, yellow)
        if result is None:
            result = self._centre_waypoints(blue, yellow)

        waypoints, direction = result
        local_path = self._drive_along(waypoints, direction)
        return [self._to_world(p) for p in local_path]

    def _to_local(self, x: float, y: float) -> Point:
        c, s = math.cos(self.car_pose.yaw), math.sin(self.car_pose.yaw)
        dx, dy = x - self.car_pose.x, y - self.car_pose.y
        return (c * dx + s * dy, -s * dx + c * dy)

    def _to_world(self, p: Point) -> Point:
        c, s = math.cos(self.car_pose.yaw), math.sin(self.car_pose.yaw)
        return (self.car_pose.x + c * p[0] - s * p[1], self.car_pose.y + s * p[0] + c * p[1])

    def _cones_local(self) -> Tuple[List[Point], List[Point]]:
        blue: List[Point] = []
        yellow: List[Point] = []
        for cone in self.cones:
            p = self._to_local(cone.x, cone.y)
            (blue if cone.color == 1 else yellow).append(p)
        blue.sort(key=_norm)
        yellow.sort(key=_norm)
        return blue, yellow

    def _centre_waypoints(self, blue: List[Point], yellow: List[Point]) -> Tuple[List[Point], Point]:
        centres: List[Tuple[Point, Point]] = []

        if blue and yellow:
            big, small = (blue, yellow) if len(blue) >= len(yellow) else (yellow, blue)
            for a in big:
                b = min(small, key=lambda o: _dist(a, o))
                mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                if all(_dist(mid, c[0]) > 0.5 for c in centres):
                    left, right = (a, b) if big is blue else (b, a)
                    g = _sub(left, right)
                    centres.append((mid, _unit((g[1], -g[0]))))

        elif blue or yellow:
            side = blue if blue else yellow
            sign = -1.0 if blue else 1.0
            if len(side) == 1:
                cone = side[0]
                centres = [((cone[0], cone[1] + sign * HALF_TRACK_WIDTH), (1.0, 0.0))]
            else:
                for i, cone in enumerate(side):
                    tangent = _unit(_sub(side[min(i + 1, len(side) - 1)], side[max(i - 1, 0)]))
                    n = _left_normal(tangent)
                    centres.append(((cone[0] + sign * HALF_TRACK_WIDTH * n[0],
                                     cone[1] + sign * HALF_TRACK_WIDTH * n[1]), tangent))

        if not centres:
            return [(0.0, 0.0)], (1.0, 0.0)

        ordered = _chain(centres)
        first, first_dir = ordered[0]
        if len(ordered) >= 2:
            first_dir = _unit(_sub(ordered[1][0], first))
        waypoints: List[Point] = [(0.0, 0.0)]
        ahead = _dot(first, first_dir)
        if ahead > 0.0:
            back = min(ENTRY_DISTANCE, ahead / 2)
            waypoints.append((first[0] - first_dir[0] * back, first[1] - first_dir[1] * back))
        waypoints += [c[0] for c in ordered]

        if len(ordered) == 1:
            direction = first_dir
        else:
            direction = _unit(_sub(waypoints[-1], waypoints[-2]))
        return waypoints, direction

    def _curve_from_three_cones(
        self, blue: List[Point], yellow: List[Point]
    ) -> Optional[Tuple[List[Point], Point]]:
        is_blue = len(blue) >= len(yellow)
        side, other = (blue, yellow) if is_blue else (yellow, blue)
        p1, p2, p3 = side[:3]

        circle = _circle_through(p1, p2, p3)
        if circle is None:
            return None
        centre, radius = circle

        turn = 1.0 if _cross(_sub(p2, p1), _sub(p3, p2)) > 0 else -1.0

        offset = HALF_TRACK_WIDTH
        if other:
            offset = sum(abs(_dist(o, centre) - radius) for o in other) / len(other) / 2

        centre_radius = radius + turn * offset if is_blue else radius - turn * offset
        if centre_radius <= 0.1:
            return None

        a1 = math.atan2(p1[1] - centre[1], p1[0] - centre[0])
        a3 = math.atan2(p3[1] - centre[1], p3[0] - centre[0])
        a_car = math.atan2(-centre[1], -centre[0])
        before = ((a1 - a_car) * turn) % (2 * math.pi)
        if before < math.pi / 2:
            a1 = a_car + turn * min(before, 2 * ENTRY_DISTANCE / centre_radius)
        sweep = ((a3 - a1) * turn) % (2 * math.pi) + MAX_ARC_ANGLE
        n = max(2, math.ceil(sweep * centre_radius / 0.1))
        arc = []
        for k in range(n + 1):
            a = a1 + turn * sweep * k / n
            arc.append((centre[0] + centre_radius * math.cos(a), centre[1] + centre_radius * math.sin(a)))

        a_end = a1 + turn * sweep
        direction = (-turn * math.sin(a_end), turn * math.cos(a_end))
        return [(0.0, 0.0)] + arc, direction

    def _drive_along(self, waypoints: List[Point], direction: Point) -> List[Point]:
        length = sum(_dist(a, b) for a, b in zip(waypoints, waypoints[1:]))
        goal = max(MIN_PATH_LENGTH, length + EXTRA_LENGTH)

        last = waypoints[-1]
        reference = _smooth(waypoints + [(last[0] + direction[0] * MAX_PATH_LENGTH,
                                          last[1] + direction[1] * MAX_PATH_LENGTH)])
        return _pure_pursuit(reference, goal)


def _sub(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1])


def _norm(a: Point) -> float:
    return math.hypot(a[0], a[1])


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _unit(a: Point) -> Point:
    n = _norm(a)
    return (a[0] / n, a[1] / n) if n > 1e-9 else (1.0, 0.0)


def _left_normal(a: Point) -> Point:
    return (-a[1], a[0])


def _cross(a: Point, b: Point) -> float:
    return a[0] * b[1] - a[1] * b[0]


def _dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _chain(centres: List[Tuple[Point, Point]]) -> List[Tuple[Point, Point]]:
    remaining = list(centres)
    ordered: List[Tuple[Point, Point]] = []
    pos: Point = (0.0, 0.0)
    heading: Optional[Point] = None
    while remaining:
        ahead = [c for c in remaining if heading is None or _dot(_sub(c[0], pos), heading) > 0]
        if not ahead:
            break
        nxt = min(ahead, key=lambda c: _dist(c[0], pos))
        heading = _unit(_sub(nxt[0], pos))
        pos = nxt[0]
        ordered.append(nxt)
        remaining.remove(nxt)
    return ordered


def _smooth(points: List[Point]) -> List[Point]:
    dense: List[Point] = [points[0]]
    for a, b in zip(points, points[1:]):
        n = max(1, math.ceil(_dist(a, b) / SMOOTH_SPACING))
        dense += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
    for _ in range(SMOOTH_ITERATIONS):
        cut: List[Point] = [dense[0]]
        for a, b in zip(dense, dense[1:]):
            cut.append((0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]))
            cut.append((0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1]))
        cut.append(dense[-1])
        dense = cut
    return dense


def _pure_pursuit(reference: List[Point], goal: float) -> List[Point]:
    s_ref = [0.0]
    for a, b in zip(reference, reference[1:]):
        s_ref.append(s_ref[-1] + _dist(a, b))

    max_curvature = 1.0 / MIN_TURN_RADIUS
    x, y, yaw = 0.0, 0.0, 0.0
    i = 0
    path: List[Point] = []
    travelled = 0.0
    next_record = STEP
    while next_record <= MAX_PATH_LENGTH + 1e-9:
        window = range(i, min(len(reference), i + 60))
        i = min(window, key=lambda j: (reference[j][0] - x) ** 2 + (reference[j][1] - y) ** 2)
        if s_ref[i] >= goal and travelled >= MIN_PATH_LENGTH - 1e-9 and path:
            break

        k = i
        while k < len(reference) - 1 and s_ref[k] - s_ref[i] < LOOKAHEAD:
            k += 1
        tx, ty = reference[k]

        alpha = (math.atan2(ty - y, tx - x) - yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(alpha) > math.pi / 2:
            curvature = math.copysign(max_curvature, alpha)
        else:
            curvature = 2 * math.sin(alpha) / _dist((x, y), (tx, ty))
            curvature = max(-max_curvature, min(max_curvature, curvature))

        ds = min(SIM_STEP, next_record - travelled)
        mid_yaw = yaw + curvature * ds / 2
        x += math.cos(mid_yaw) * ds
        y += math.sin(mid_yaw) * ds
        yaw += curvature * ds
        travelled += ds
        if travelled >= next_record - 1e-9:
            path.append((x, y))
            next_record += STEP
    return path


def _circle_through(p1: Point, p2: Point, p3: Point) -> Optional[Tuple[Point, float]]:
    (ax, ay), (bx, by), (cx, cy) = p1, p2, p3
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-9:
        return None
    a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    radius = math.hypot(ax - ux, ay - uy)
    if radius > MAX_CIRCLE_RADIUS:
        return None
    return (ux, uy), radius
