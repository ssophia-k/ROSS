import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial import Voronoi

from ross.core.environment import Field
from ross.core.swarm import Swarm
from ross.core.simulator import Simulator
from ross.behaviors.voronoi_balance import VoronoiBalanceBehavior

# Initialize Environment, Swarm, Simulator

field = Field(width=50, height=50, num_points=7)
swarm = Swarm(field=field, num_robots=50, sensing_radius=50.0, behavior_cls=VoronoiBalanceBehavior)
sim = Simulator(field=field, swarm=swarm, steps=50)

# Set up Plot

fig, ax = plt.subplots()
ax.set_xlim(0, field.width)
ax.set_ylim(0, field.height)
ax.set_aspect("equal")
ax.set_title("Swarm Robots with Voronoi Diagram")
ax.set_xlabel("X")
ax.set_ylabel("Y")

# Plot seed points as black triangles
seed_points = field.get_all_positions()
ax.plot(seed_points[:, 0], seed_points[:, 1], '^', markersize=6, color='black', label="Seed Points")

# Assign unique colors to robots
num_robots = len(swarm.robots)
cmap = plt.colormaps.get_cmap('tab20')
colors = cmap.resampled(num_robots)

# Create scatter and trail handles
scatters = [ax.plot([], [], 'o', markersize=4, color=colors(i))[0] for i in range(num_robots)]
trails = [ax.plot([], [], '-', linewidth=1, color=colors(i), alpha=0.25)[0] for i in range(num_robots)]

# Voronoi line storage
voronoi_lines = []

# Frame count label
frame_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, fontsize=10,
                     verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7))

# Voronoi Drawing Function

def draw_voronoi(points):
    global voronoi_lines
    for line in voronoi_lines:
        line.remove()
    voronoi_lines = []

    if len(points) < 2:
        return

    vor = Voronoi(points)
    center = points.mean(axis=0)
    radius = np.ptp(points, axis=0).max() * 2

    for pointidx, ridge_vertices in zip(vor.ridge_points, vor.ridge_vertices):
        ridge_vertices = np.asarray(ridge_vertices)
        if np.all(ridge_vertices >= 0):
            p1, p2 = vor.vertices[ridge_vertices]
            line, = ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'lightgray', lw=0.8, zorder=0)
        else:
            i = ridge_vertices[ridge_vertices >= 0][0]
            t = vor.points[pointidx[1]] - vor.points[pointidx[0]]
            t /= np.linalg.norm(t)
            n = np.array([-t[1], t[0]])
            midpoint = vor.points[pointidx].mean(axis=0)
            direction = np.sign(np.dot(midpoint - center, n)) * n
            far_point = vor.vertices[i] + direction * radius
            line, = ax.plot([vor.vertices[i, 0], far_point[0]],
                            [vor.vertices[i, 1], far_point[1]],
                            'lightgray', lw=0.8, zorder=0)
        voronoi_lines.append(line)

# Animation Frame Update

def update(frame):
    sim.step()

    for i, bot in enumerate(swarm.robots):
        path = sim.robot_paths[i]
        xdata, ydata = zip(*path)
        scatters[i].set_data([xdata[-1]], [ydata[-1]])
        trails[i].set_data(xdata, ydata)

    draw_voronoi(seed_points)
    frame_text.set_text(f"Frame: {frame + 1}/50")

    # final frame has no trails
    if frame == sim.steps - 1:
        for trail in trails:
            trail.set_data([], [])

    return scatters + trails + voronoi_lines + [frame_text]

# Run Animation

ani = FuncAnimation(fig, update, frames=50, interval=100, blit=True, repeat=False)
plt.legend()
plt.tight_layout()
plt.show()
fig.savefig("voronoi_swarm_simulation.png", dpi=300, bbox_inches='tight')
