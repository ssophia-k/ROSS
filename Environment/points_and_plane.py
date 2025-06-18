import numpy as np
import matplotlib.pyplot as plt

class _Point:
    def __init__(self, x: float, y: float, id: int):
        self.x = x
        self.y = y
        self.id = id

    def move(self, dx: float, dy: float):
        self.x += dx
        self.y += dy

    def set_position(self, x: float, y: float):
        self.x = x
        self.y = y

    def get_position(self):
        return (self.x, self.y)


class Plane:
    def __init__(self, width: int, height: int, num_points: int = 0):
        self.width = width
        self.height = height
        self._points = {}       # id -> _Point
        self._next_id = 0
        self._generate_random_points(num_points)

    def _generate_random_points(self, n):
        for _ in range(n):
            x, y = np.random.rand(2) * [self.width, self.height]
            self._add_point_internal(x, y)

    def _add_point_internal(self, x: float, y: float):
        p = _Point(x, y, self._next_id)
        self._points[self._next_id] = p
        self._next_id += 1

    def add_point(self, x: float, y: float):
        """Public method to add a new point at (x, y)."""
        self._add_point_internal(x, y)

    def remove_point(self, id: int):
        """Remove a point by its ID."""
        if id in self._points:
            del self._points[id]

    def move_point(self, id: int, dx: float, dy: float):
        """Move a point by (dx, dy)."""
        if id in self._points:
            self._points[id].move(dx, dy)

    def set_point_position(self, id: int, x: float, y: float):
        """Set the point to a new absolute position."""
        if id in self._points:
            self._points[id].set_position(x, y)

    def get_all_positions(self):
        """Get all point positions as a NumPy array."""
        return np.array([p.get_position() for p in self._points.values()])

    def plot(self):
        """Plot all points on the 2D plane."""
        positions = self.get_all_positions()
        if positions.size == 0:
            return
        plt.figure(figsize=(6, 6))
        plt.scatter(positions[:, 0], positions[:, 1], c='blue', s=50)
        plt.xlim(0, self.width)
        plt.ylim(0, self.height)
        plt.grid(True)
        plt.title("Points on Plane")
        plt.show()
