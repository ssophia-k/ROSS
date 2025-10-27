import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib as mpl
from scipy.spatial import Voronoi
from typing import Optional, List, Dict, Any, Tuple

# ===============================
# Environment and Point classes
# ===============================
class Point:
    """Represents a point in 2D space with a unique identifier."""

    def __init__(self, x: float, y: float, point_id: int):
        self.x = x
        self.y = y
        self.point_id = point_id

    def move(self, dx: float, dy: float) -> None:
        self.x += dx
        self.y += dy

    def set_position(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def get_position(self) -> Tuple[float, float]:
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
            w_min = max(0, w_min)
            w_max = min(width, w_max)
        if w_min > w_max:
            raise ValueError(
                f"Invalid spawnable_width_interval: "
                f"after clamping to [0, {width}] got ({w_min}, {w_max})"
            )
        self.spawnable_width_interval: tuple[float, float] = (w_min, w_max)

        # height interval within which points can be spawned
        if spawnable_height_interval is None:
            h_min, h_max = 0, height
        else:
            h_min, h_max = spawnable_height_interval
            h_min = max(0, h_min)
            h_max = min(height, h_max)
        if h_min > h_max:
            raise ValueError(
                f"Invalid spawnable_height_interval: "
                f"after clamping to [0, {height}] got ({h_min}, {h_max})"
            )
        self.spawnable_height_interval: tuple[float, float] = (h_min, h_max)

        self._points: dict[int, Point] = {}
        self._next_id = 0
        self._generate_random_points(num_points)

    def _generate_random_points(self, count: int) -> None:
        w_min, w_max = self.spawnable_width_interval
        h_min, h_max = self.spawnable_height_interval
        for _ in range(count):
            x = np.random.uniform(w_min, w_max)
            y = np.random.uniform(h_min, h_max)
            self._add_point_internal(x, y)

    def _add_point_internal(self, x: float, y: float) -> None:
        point = Point(x, y, self._next_id)
        self._points[self._next_id] = point
        self._next_id += 1

    def add_point(self, x: float, y: float) -> int:
        self._add_point_internal(x, y)
        return self._next_id - 1

    def remove_point(self, point_id: int) -> None:
        self._points.pop(point_id, None)

    def move_point(self, point_id: int, dx: float, dy: float) -> None:
        point = self._points.get(point_id)
        if point:
            point.move(dx, dy)

    def set_point_position(self, point_id: int, x: float, y: float) -> None:
        point = self._points.get(point_id)
        if point:
            point.set_position(x, y)

    def get_point(self, point_id: int) -> Point | None:
        return self._points.get(point_id)

    def get_all_positions(self) -> np.ndarray:
        return np.array([point.get_position() for point in self._points.values()])


# ===============================
# Robot
# ===============================
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
        self.swarm = None  # to be set by Swarm.add_robot()
        self.field = field
        self.sensing_radius = sensing_radius
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
        return {
            'robots': self.sense_robots(),
            'voronoi_points': self.sense_voronoi_points()
        }

    def send_message(self, recipient_id: int, content: dict) -> None:
        if self.swarm:
            self.swarm.deliver_message(self.id, recipient_id, content)

    def receive_message(self, sender_id: int, content: dict) -> None:
        self.messages.append({'from': sender_id, 'content': content})

    def get_messages(self) -> list:
        msgs = self.messages[:]
        self.messages.clear()
        return msgs

    def __repr__(self) -> str:
        return f"<Robot id={self.id}, pos=({self.x:.2f}, {self.y:.2f})>"


# ===============================
# Swarm
# ===============================
class Swarm:
    def __init__(self,
                 field: Field,
                 spawnable_width_interval: tuple[float, float] | None = None,
                 spawnable_height_interval: tuple[float, float] | None = None,
                 num_robots: int = 10,
                 sensing_radius: float = 25.0,
                 behavior_cls=None):
        self.field = field
        self.robots = []
        if behavior_cls is None:
            raise ValueError("behavior_cls must be provided")
        self.behavior = behavior_cls()

        # spawn ranges for robots
        if spawnable_width_interval is None:
            w_min, w_max = 0, field.width
        else:
            w_min, w_max = spawnable_width_interval
            w_min = max(0, w_min)
            w_max = min(field.width, w_max)
        if w_min > w_max:
            raise ValueError(
                f"Invalid spawnable_width_interval: "
                f"after clamping to [0, {field.width}] got ({w_min}, {w_max})"
            )
        self.spawnable_width_interval: tuple[float, float] = (w_min, w_max)

        if spawnable_height_interval is None:
            h_min, h_max = 0, field.height
        else:
            h_min, h_max = spawnable_height_interval
            h_min = max(0, h_min)
            h_max = min(field.height, h_max)
        if h_min > h_max:
            raise ValueError(
                f"Invalid spawnable_height_interval: "
                f"after clamping to [0, {field.height}] got ({h_min}, {h_max})"
            )
        self.spawnable_height_interval: tuple[float, float] = (h_min, h_max)

        for i in range(num_robots):
            x = np.random.uniform(w_min, w_max)
            y = np.random.uniform(h_min, h_max)
            bot = Robot(x, y, robot_id=i, field=field, sensing_radius=sensing_radius)
            bot.set_swarm(self)
            self.robots.append(bot)

    def step(self):
        scans = {bot.id: bot.scan() for bot in self.robots}
        self.behavior.send_messages(self.robots, scans)
        self.behavior.apply_movement(self.robots, scans)

    def deliver_message(self, sender_id: int, recipient_id: int, content: dict) -> None:
        for bot in self.robots:
            if bot.id == recipient_id:
                bot.receive_message(sender_id, content)
                break


# ===============================
# Simulator
# ===============================
class Simulator:
    def __init__(self, field, swarm, steps=1000):
        self.field = field
        self.swarm = swarm
        self.steps = steps
        self.robot_paths = [[] for _ in range(len(swarm.robots))]
        for i, bot in enumerate(self.swarm.robots):
            self.robot_paths[i].append(bot.get_position())

    def step(self):
        self.swarm.step()
        for i, bot in enumerate(self.swarm.robots):
            self.robot_paths[i].append(bot.get_position())

    def run(self):
        for _ in range(self.steps):
            self.step()

    def get_paths(self):
        return self.robot_paths


# ===============================
# Behavior: VoronoiBalance
# ===============================
class VoronoiBalanceBehavior:
    def apply_movement(self, robots, scans):
        for bot in robots:
            data = scans[bot.id]
            pts = data['voronoi_points']
            nbrs = data['robots']

            messages = bot.get_messages()
            for msg in messages:
                if msg['content']['type'] == 'distance_info':
                    sender_id = msg['from']
                    receiver_dict = next((nbr for nbr in nbrs if nbr['robot_id'] == sender_id), None)
                    if receiver_dict is None:
                        continue
                    else:
                        sender_to_receiver_vector = receiver_dict['vector']

                    pts_in_sender_frame = msg['content']['voronoi_points']
                    pts_in_receiver_frame = []
                    for pt in pts_in_sender_frame:
                        transformed_pt = pt['vector'] + sender_to_receiver_vector
                        pts_in_receiver_frame.append({
                            'point_id': pt['point_id'],
                            'position': pt['position'],
                            'vector': transformed_pt,
                            'distance': np.linalg.norm(transformed_pt)
                        })

                    nbrs_in_sender_frame = msg['content']['neighbors']
                    nbrs_in_receiver_frame = []
                    for nbr in nbrs_in_sender_frame:
                        transformed_nbr = nbr['vector'] + sender_to_receiver_vector
                        nbrs_in_receiver_frame.append({
                            'robot_id': nbr['robot_id'],
                            'vector': transformed_nbr,
                            'distance': np.linalg.norm(transformed_nbr)
                        })

                    for new_pt in pts_in_receiver_frame:
                        match = next((existing_pt for existing_pt in pts if new_pt['point_id'] == existing_pt['point_id']), None)
                        if match:
                            pass
                        else:
                            pts.append(new_pt)

                    for new_nbr in nbrs_in_receiver_frame:
                        match = next((existing_nbr for existing_nbr in nbrs if new_nbr['robot_id'] == existing_nbr['robot_id']), None)
                        if match:
                            pass
                        else:
                            nbrs.append(new_nbr)

            neighbor_repel = np.zeros(2)
            for nbr in nbrs:
                d = nbr['distance']
                if d < 1e-6:
                    continue
                elif d < bot.sensing_radius/2:
                    dirn = nbr['vector'] / d
                    neighbor_repel -= dirn * (1.0 / d**2)
            neighbor_repel *= 5

            sorted_pts = sorted(pts, key=lambda d: d['distance'])
            point_drive = np.zeros(2)
            if len(sorted_pts) >= 2:
                sorted_pts = sorted_pts[:2]
                u_closest = sorted_pts[0]['vector'] / sorted_pts[0]['distance']
                u_farthest = sorted_pts[-1]['vector'] / sorted_pts[-1]['distance']
                point_drive = (u_farthest - u_closest)
            norm = np.linalg.norm(point_drive)
            if norm > 1e-6:
                point_drive /= norm

            move_vec = point_drive + neighbor_repel
            if np.linalg.norm(move_vec) > 1e-2:
                bot.move_toward(bot.get_position() + move_vec/np.linalg.norm(move_vec))

    def send_messages(self, robots, scans):
        for bot in robots:
            data = scans[bot.id]
            pts = data['voronoi_points']
            nbrs = data['robots']
            for nbr in nbrs:
                bot.send_message(
                    recipient_id=nbr['robot_id'],
                    content={
                        'type': 'distance_info',
                        'from': bot.id,
                        'to': nbr['robot_id'],
                        'voronoi_points': pts,
                        'neighbors': nbrs
                    }
                )


# ===============================
# Visualization helpers
# ===============================

def intersection(line_pt_1, line_pt_2, ray_start, ray_direction):
    """Calculate the intersection of a line segment and a ray."""
    line_vector = line_pt_2 - line_pt_1
    ray_vector = ray_direction

    if abs(np.cross(line_vector, ray_vector)) < 1e-6:
        return None  # Lines are parallel

    A = np.array([[line_vector[0], -ray_vector[0]], [line_vector[1], -ray_vector[1]]])
    B = np.array([ray_start[0] - line_pt_1[0], ray_start[1] - line_pt_1[1]])
    t1, t2 = np.linalg.solve(A, B)
    if t1 < 0 or t2 < 0:
        return None  # No intersection or intersection is outside the segment
    elif t1 > 1:
        return None  # Intersection is beyond the segment length
    else:
        return line_pt_1 + t1 * line_vector


def draw_voronoi(points, field, ax, voronoi_lines_store):
    xmin = 0
    xmax = field.width
    ymin = 0
    ymax = field.height

    for line in voronoi_lines_store:
        line.remove()
    voronoi_lines_store.clear()

    if len(points) < 2:
        return

    vor = Voronoi(points)

    for pointidx, ridge_vertices in zip(vor.ridge_points, vor.ridge_vertices):
        ridge_vertices = np.asarray(ridge_vertices)
        if np.all(ridge_vertices >= 0):
            p1, p2 = vor.vertices[ridge_vertices]
            line, = ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'lightgray', lw=0.8, zorder=0)
        else:
            i = ridge_vertices[ridge_vertices >= 0][0]
            vertex = vor.vertices[i]
            t = vor.points[pointidx[1]] - vor.points[pointidx[0]]
            t /= np.linalg.norm(t)
            n = np.array([-t[1], t[0]])
            midpoint = vor.points[pointidx].mean(axis=0)
            direction = np.sign(np.dot(midpoint - points.mean(axis=0), n)) * n

            left = intersection(np.array([xmin, ymin]), np.array([xmin, ymax]), midpoint, direction)
            right = intersection(np.array([xmax, ymin]), np.array([xmax, ymax]), midpoint, direction)
            bottom = intersection(np.array([xmin, ymin]), np.array([xmax, ymin]), midpoint, direction)
            top = intersection(np.array([xmin, ymax]), np.array([xmax, ymax]), midpoint, direction)

            candidates = [p for p in (left, right, bottom, top) if p is not None]
            if not candidates:
                continue
            dists = [np.linalg.norm(p - vertex) for p in candidates]
            far_point = candidates[np.argmax(dists)]

            line, = ax.plot([vor.vertices[i, 0], far_point[0]],
                            [vor.vertices[i, 1], far_point[1]],
                            'lightgray', lw=0.8, zorder=0)

        voronoi_lines_store.append(line)


# ===============================
# Main: animate and save
# ===============================

def main():
    STEP_COUNT = 200  # Number of simulation steps
    ANIMATION_INTERVAL = 200  # milliseconds per frame

    # Initialize Environment, Swarm, Simulator
    field = Field(width=100, height=100,
                  spawnable_width_interval=(20, 80), spawnable_height_interval=(20, 80),
                  num_points=7)

    swarm = Swarm(field=field,
                  spawnable_width_interval=(40, 50), spawnable_height_interval=(40, 50),
                  num_robots=50, sensing_radius=20.0,
                  behavior_cls=VoronoiBalanceBehavior)

    sim = Simulator(field=field, swarm=swarm, steps=STEP_COUNT)

    # Set up Plot
    fig, ax = plt.subplots()
    ax.set_xlim(0, field.width)
    ax.set_ylim(0, field.height)
    ax.set_aspect("equal")

    ax.set_title("Swarm Robots with Voronoi Diagram" +
                 "\nNumber of Robots: " + str(len(swarm.robots)) +
                 "\nSensing Radius: " + str(swarm.robots[0].sensing_radius))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    # Plot seed points as black triangles
    seed_points = field.get_all_positions()
    ax.plot(seed_points[:, 0], seed_points[:, 1], '^', markersize=6, color='black', label="Seed Points")

    # Assign unique colors to robots
    num_robots = len(swarm.robots)
    cmap = mpl.colormaps.get_cmap('tab20')
    colors = cmap.resampled(num_robots)

    # Create scatter and trail handles
    scatters = [ax.plot([], [], 'o', markersize=4, color=colors(i))[0] for i in range(num_robots)]
    trails = [ax.plot([], [], '-', linewidth=1, color=colors(i), alpha=0.25)[0] for i in range(num_robots)]

    # Voronoi line storage
    voronoi_lines = []

    # Frame count label
    frame_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, fontsize=10,
                         verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7))

    def update(frame):
        sim.step()
        for i, bot in enumerate(swarm.robots):
            path = sim.robot_paths[i]
            xdata, ydata = zip(*path)
            scatters[i].set_data([xdata[-1]], [ydata[-1]])
            trails[i].set_data(xdata, ydata)
        draw_voronoi(seed_points, field, ax, voronoi_lines)
        frame_text.set_text(f"Frame: {frame + 1}/{STEP_COUNT}")
        if frame == sim.steps - 1:
            for trail in trails:
                trail.set_data([], [])
        return scatters + trails + voronoi_lines + [frame_text]

    ani = FuncAnimation(fig, update, frames=STEP_COUNT, interval=ANIMATION_INTERVAL, blit=True, repeat=False)
    ax.legend(loc='lower right')
    plt.tight_layout()

    os.makedirs("videos", exist_ok=True)
    ani.save("videos/voronoi_swarm_simulation.gif", fps=int(1000/ANIMATION_INTERVAL))
    fig.savefig("voronoi_swarm_simulation.png", dpi=300, bbox_inches='tight')

    # Optionally show the plot interactively
    # plt.show()


if __name__ == "__main__":
    main()
