from __future__ import annotations

from typing import List

import matplotlib.pyplot as plt

from src.models import CarPose, Cone, Path2D
from src.path_planning import PathPlanning


class PathTester:
    def __init__(self, cones: List[Cone], car_pose: CarPose):
        self.cones = cones
        self.car_pose = car_pose

    def run(self) -> Path2D:
        planner = PathPlanning(self.car_pose, self.cones)
        path = planner.generatePath()

        self._plot_scene(path)
        return path

    def _plot_scene(self, path: Path2D) -> None:
        _, ax = plt.subplots(figsize=(8, 6))
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, linestyle=":", linewidth=0.5)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title("FSAI-Style Cone Track Path Planning Test")
        ax.set_xlim(-1.0, 6.0)
        ax.set_ylim(-1.0, 6.0)

        yellow_x = [c.x for c in self.cones if c.color == 0]
        yellow_y = [c.y for c in self.cones if c.color == 0]
        blue_x = [c.x for c in self.cones if c.color == 1]
        blue_y = [c.y for c in self.cones if c.color == 1]

        handles = []
        if yellow_x:
            h_y = ax.scatter(yellow_x, yellow_y, c="gold", edgecolors="black", label="Yellow (Right)")
            handles.append(h_y)
        if blue_x:
            h_b = ax.scatter(blue_x, blue_y, c="royalblue", edgecolors="black", label="Blue (Left)")
            handles.append(h_b)

        ax.scatter([self.car_pose.x], [self.car_pose.y], c="red", s=60, marker="o", label="Car")
        self._draw_heading_arrow(ax)

        if path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            ax.plot(px, py, "-", color="limegreen", linewidth=2.0, label="Planned Path")

        if handles:
            ax.legend(loc="best")

        plt.show()

    def _draw_heading_arrow(self, ax) -> None:
        import math

        length = 1.0
        dx = math.cos(self.car_pose.yaw) * length
        dy = math.sin(self.car_pose.yaw) * length
        ax.arrow(self.car_pose.x, self.car_pose.y, dx, dy, head_width=0.3, head_length=0.4, fc="red", ec="red")
