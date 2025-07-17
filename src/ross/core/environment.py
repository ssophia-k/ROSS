import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, Tuple, Optional


class Point:
    """Represents a point in 2D space with a unique identifier."""

    def __init__(self, x: float, y: float, point_id: int):
        self.x = x
        self.y = y
        self.point_id = point_id

    def move(self, dx: float, dy: float) -> None:
        """Move the point by a relative amount."""
        self.x += dx
        self.y += dy

    def set_position(self, x: float, y: float) -> None:
        """Set the point's absolute position."""
        self.x = x
        self.y = y

    def get_position(self) -> Tuple[float, float]:
        """Get the current position of the point."""
        return self.x, self.y

    def __repr__(self) -> str:
        return f"Point(id={self.point_id}, x={self.x:.2f}, y={self.y:.2f})"


class Field:
    """Represents a 2D field containing a collection of seed points."""

    def __init__(self, 
                 width: int, 
                 height: int, 
                 spawnable_width_interval: tuple[float, float] | None = None, 
                 spawnable_height_interval: tuple[float, float] | None = None, 
                 num_points: int = 0):
        
        self.width = width
        self.height = height

        # width interval within which points can be spawned
        if spawnable_width_interval is None:
            w_min, w_max = 0, width
        else:
            w_min, w_max = spawnable_width_interval
            # clamp into [0, width]
            w_min = max(0, w_min)
            w_max = min(width, w_max)
        if w_min > w_max:
            raise ValueError(
                f"Invalid spawnable_width_interval: "
                f"after clamping to [0, {width}] got ({w_min}, {w_max})"
            )
        self.spawnable_width_interval: tuple[int, int] = (w_min, w_max)
        
        # height interval within which points can be spawned
        if spawnable_height_interval is None:
            h_min, h_max = 0, height
        else:
            h_min, h_max = spawnable_height_interval
            # clamp into [0, height]
            h_min = max(0, h_min)
            h_max = min(height, h_max)
        if h_min > h_max:
            raise ValueError(
                f"Invalid spawnable_height_interval: "
                f"after clamping to [0, {height}] got ({h_min}, {h_max})"
            )
        self.spawnable_height_interval: tuple[int, int] = (h_min, h_max)

        self._points: dict[int, Point] = {}
        self._next_id = 0
        self._generate_random_points(num_points)


    def _generate_random_points(self, count: int) -> None:
        """Generate a specified number of random points within the field."""
        w_min, w_max = self.spawnable_width_interval
        h_min, h_max = self.spawnable_height_interval
        for _ in range(count):
            x = np.random.uniform(w_min, w_max)
            y = np.random.uniform(h_min, h_max)
            self._add_point_internal(x, y)


    def _add_point_internal(self, x: float, y: float) -> None:
        """Add a point internally and assign a unique ID."""
        point = Point(x, y, self._next_id)
        self._points[self._next_id] = point
        self._next_id += 1


    def add_point(self, x: float, y: float) -> int:
        """Add a new point at (x, y) and return its assigned ID."""
        self._add_point_internal(x, y)
        return self._next_id - 1
    

    def remove_point(self, point_id: int) -> None:
        """Remove a point by its ID if it exists."""
        self._points.pop(point_id, None)


    def move_point(self, point_id: int, dx: float, dy: float) -> None:
        """Move the point with the given ID by a delta (dx, dy)."""
        point = self._points.get(point_id)
        if point:
            point.move(dx, dy)


    def set_point_position(self, point_id: int, x: float, y: float) -> None:
        """Set the point's position to the new coordinates."""
        point = self._points.get(point_id)
        if point:
            point.set_position(x, y)


    def get_point(self, point_id: int) -> Point | None:
        """Return the Point with the given ID, or None if not found."""
        return self._points.get(point_id)
    

    def get_all_positions(self) -> np.ndarray:
        """Return all point positions as a NumPy array of shape (n, 2)."""
        return np.array([point.get_position() for point in self._points.values()])
    

    def __iter__(self):
        """Allow iteration over all Point objects in the field."""
        return iter(self._points.values())
    

    def plot(self, show_ids: bool = True) -> None:
        """Plot all points in the field."""
        positions = self.get_all_positions()
        if positions.size == 0:
            print("No points to plot.")
            return

        plt.figure(figsize=(6, 6))
        plt.scatter(positions[:, 0], positions[:, 1], c="blue", s=60, edgecolor="black")

        if show_ids:
            for point in self._points.values():
                plt.text(
                    point.x,
                    point.y,
                    str(point.point_id),
                    fontsize=9,
                    ha="right",
                    va="bottom",
                )

        plt.title("Points in Field")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.xlim(0, self.width)
        plt.ylim(0, self.height)
        plt.grid(True)
        plt.gca().set_aspect("equal", adjustable="box")
        plt.tight_layout()
        plt.show()
