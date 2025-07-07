import numpy as np
from typing import Optional, List, Dict, Any

class Robot:
    def __init__(
        self,
        x: float,
        y: float,
        robot_id: Optional[int] = None,
        field: Optional[Any] = None,
        sensing_radius: float = 10.0
    ):
        self.x = x
        self.y = y
        self.id = robot_id

        # World references
        self.swarm = None  # to be set by Swarm.add_robot()
        self.field = field

        # Sensing
        self.sensing_radius = sensing_radius

        # Messaging
        self.messages = []  # Message queue

    def set_swarm(self, swarm: Any) -> None:
        self.swarm = swarm

    def get_position(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=np.float64)

    def move_toward(self, target: np.ndarray, speed: float = 1.0) -> None:
        direction = target - self.get_position()
        distance = np.linalg.norm(direction)
        if distance > 0:
            step = (direction / distance) * speed
            self.x += step[0]
            self.y += step[1]

        if self.field:
            self.x = max(0, min(self.x, self.field.width))
            self.y = max(0, min(self.y, self.field.height))

    def sense_robots(self) -> List[Dict[str, Any]]:
        """Return vectors from other nearby robots to self."""
        if not self.swarm:
            return []

        position = self.get_position()
        readings = []

        for other in self.swarm.robots:
            if other is self:
                continue
            delta = other.get_position() - position
            distance = np.linalg.norm(delta)
            if distance <= self.sensing_radius:
                readings.append({
                    'robot_id': other.id,
                    'vector': delta,
                    'distance': distance
                })

        return readings

    def sense_voronoi_points(self) -> List[Dict[str, Any]]:
        """Return vectors from Voronoi points to self."""
        if not self.field:
            return []

        position = self.get_position()
        readings = []

        for pid, point in self.field._points.items():
            point_pos = np.array(point.get_position(), dtype=np.float64)
            delta = point_pos - position
            distance = np.linalg.norm(delta)
            if distance <= self.sensing_radius:
                readings.append({
                    'point_id': pid,
                    'position': tuple(point_pos),
                    'vector': delta,
                    'distance': distance
                })

        return readings

    def scan(self) -> Dict[str, List[Dict[str, Any]]]:
        """Perform a full sensor scan of nearby robots and Voronoi points."""
        return {
            'robots': self.sense_robots(),
            'voronoi_points': self.sense_voronoi_points()
        }

    def send_message(self, recipient_id: int, content: dict) -> None:
        """Send a message to another robot in the swarm."""
        if self.swarm:
            self.swarm.deliver_message(self.id, recipient_id, content)

    def receive_message(self, sender_id: int, content: dict) -> None:
        """Receive a message from another robot."""
        self.messages.append({'from': sender_id, 'content': content})

    def get_messages(self) -> list:
        """Retrieve and clear all received messages."""
        msgs = self.messages[:]
        self.messages.clear()
        return msgs

    def __repr__(self) -> str:
        return f"<Robot id={self.id}, pos=({self.x:.2f}, {self.y:.2f})>"
