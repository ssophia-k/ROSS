import numpy as np
from ross.environment.points_and_plane import Plane
from ross.agents.robot import Robot

class Swarm:
    def __init__(self,
                 plane: Plane,
                 num_robots: int,
                 sensing_radius: float = 10.0):
        self.plane = plane
        self.robots = []
        self.sensing_radius = sensing_radius

        # Create robots at random positions on the plane
        for i in range(num_robots):
            x = np.random.rand() * plane.width
            y = np.random.rand() * plane.height
            bot = Robot(x, y, id=i, plane=plane,
                        sensing_radius=sensing_radius)
            bot.set_swarm(self)
            self.robots.append(bot)

    def step(self):
        scans = {bot.id: bot.scan() for bot in self.robots}
        self.apply_logic(scans)

    def apply_logic(self, scans):
        for bot in self.robots:
            data = scans[bot.id]
            pts  = data['voronoi_points']
            nbrs = data['robots']

            # need at least two points
            if len(pts) < 2:
                continue

            # points sorted st smallest distance is first
            nearby_points = sorted(pts, key=lambda d: d['distance'], reverse=False)
            
            # move away from the closest point. and move towards farthest point. get unit vector
            u_closest = nearby_points[0]['vector'] / nearby_points[0]['distance']
            u_farthest = nearby_points[-1]['vector'] / nearby_points[-1]['distance']
            point_drive = (u_farthest - u_closest) / np.linalg.norm(u_farthest - u_closest)

            # neightbor repulsion
            neighbor_repel = np.zeros(2)
            for nbr in nbrs:
                d = nbr['distance']
                if d < 1e-6:
                    continue
                dirn = nbr['vector'] / nbr['distance'] #np.array([math.cos(nbr['bearing']), math.sin(nbr['bearing'])])
                neighbor_repel -= dirn * (1.0 / d**2)
            neighbor_repel *= 10  # tune this gain to spread more or less

            # move one unit step
            move_vec = point_drive + neighbor_repel
            norm = np.linalg.norm(move_vec)
            if norm > 1e-2: # tolerancing for numerical stability
                step = (move_vec / norm)
                bot.move_toward(bot.get_position() + step)




    def run(self, steps: int):
        for _ in range(steps):
            self.step()
