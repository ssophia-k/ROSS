import numpy as np

class Robot:
    def __init__(self,
                 x: float,
                 y: float,
                 id: int = None,
                 plane=None,
                 sensing_radius: float = 10.0):
        self.x = x
        self.y = y
        self.id = id

        # references to the world (swarm is set by Swarm.add_robot)
        self.swarm = None
        self.plane = plane

        # sensing configuration
        self.sensing_radius = sensing_radius

    def set_swarm(self, swarm):
        self.swarm = swarm

    def get_position(self) -> np.ndarray:
        return np.array([self.x, self.y])

    def move_toward(self, target: np.ndarray, speed: float = 1.0):
        direction = target - self.get_position()
        norm = np.linalg.norm(direction)
        if norm > 0:
            step = (direction / norm) * speed
            self.x += step[0]
            self.y += step[1]

    def sense_robots(self):
        readings = []
        if not self.swarm:
            return readings

        p0 = self.get_position()
        for other in self.swarm.robots:
            if other is self:
                continue
            delta = other.get_position() - p0
            if np.linalg.norm(delta) <= self.sensing_radius:
                readings.append({
                    'robot_id': other.id,
                    'vector': delta,
                    'distance': np.linalg.norm(delta)
                })
        return readings

    def sense_voronoi_points(self):
        readings = []
        if not self.plane:
            return readings

        p0 = self.get_position()
        for pid, pt in self.plane._points.items():
            x, y = pt.get_position()
            delta = np.array([x, y]) - p0
            if np.linalg.norm(delta) <= self.sensing_radius:
                readings.append({
                    'point_id': pid,
                    'position': (x, y),
                    'vector': delta,
                    'distance': np.linalg.norm(delta)
                })
        return readings

    def scan(self):
        return {
            'robots': self.sense_robots(),
            'voronoi_points': self.sense_voronoi_points()
        }
