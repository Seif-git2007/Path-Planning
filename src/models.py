from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple


@dataclass(frozen=True)
class Cone:
    x: float
    y: float
    color: int


@dataclass(frozen=True)
class CarPose:
    x: float
    y: float
    yaw: float


Path2D = List[Tuple[float, float]]
