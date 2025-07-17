import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial import Voronoi

from ross.core.environment import Field
from ross.core.swarm import Swarm
from ross.core.simulator import Simulator
from ross.behaviors.voronoi_balance import VoronoiBalanceBehavior

STEP_COUNT = 200 # Number of simulation steps
ANIMATION_INTERVAL = 200 # milliseconds per frame

# Initialize Environment, Swarm, Simulator
field = Field(width=100, height=100, 
              spawnable_width_interval=(20,80), spawnable_height_interval=(20,80), 
              num_points=7)

swarm = Swarm(field=field, 
              spawnable_width_interval=(40,50), spawnable_height_interval=(40,50), 
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


def intersection(line_pt_1, line_pt_2, ray_start, ray_direction):
    """Calculate the intersection of a line segment and a ray."""
    line_vector = line_pt_2 - line_pt_1
    ray_vector = ray_direction

    if abs(np.cross(line_vector, ray_vector)) < 1e-6:
        return None # Lines are parallel
    
    A = np.array([[line_vector[0], -ray_vector[0]], [line_vector[1], -ray_vector[1]]])
    B = np.array([ray_start[0] - line_pt_1[0], ray_start[1] - line_pt_1[1]])
    t1, t2 = np.linalg.solve(A, B)
    if t1 < 0 or t2 < 0:
        return None # No intersection or intersection is outside the segment
    elif t1>1:
        return None  # Intersection is beyond the segment length:
    else:
        return line_pt_1 + t1 * line_vector


# Voronoi drawing function
def draw_voronoi(points, field):
    global voronoi_lines
    xmin = 0
    xmax = field.width
    ymin = 0
    ymax = field.height

    for line in voronoi_lines:
        line.remove()
    voronoi_lines = []

    if len(points) < 2:
        return

    vor = Voronoi(points)
    
    '''
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
    '''

    for pointidx, ridge_vertices in zip(vor.ridge_points, vor.ridge_vertices):
        ridge_vertices = np.asarray(ridge_vertices)
        if np.all(ridge_vertices >= 0):
            p1, p2 = vor.vertices[ridge_vertices]
            line, = ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'lightgray', lw=0.8, zorder=0)
        else:
            # Find the one valid vertex index
            i = ridge_vertices[ridge_vertices >= 0][0]
            vertex = vor.vertices[i]                # <— your start point
            # tangent between seeds, normalize, get normal
            t = vor.points[pointidx[1]] - vor.points[pointidx[0]]
            t /= np.linalg.norm(t)
            n = np.array([-t[1], t[0]])
            midpoint = vor.points[pointidx].mean(axis=0)
            # pick normal direction that points outward (use mean of points to determine "inner" center)
            direction = np.sign(np.dot(midpoint - points.mean(axis=0), n)) * n

            # Clip the ray from 'vertex' against each of the 4 edges
            left   = intersection(np.array([xmin, ymin]), np.array([xmin, ymax]), midpoint, direction)
            right  = intersection(np.array([xmax, ymin]), np.array([xmax, ymax]), midpoint, direction)
            bottom = intersection(np.array([xmin, ymin]), np.array([xmax, ymin]), midpoint, direction)
            top    = intersection(np.array([xmin, ymax]), np.array([xmax, ymax]), midpoint, direction)

            # pick the valid intersection farthest from the vertex
            # honestly there should only ever be one valid intersection
            # but just in case it lands perfectly on a corner, or if there is some numerical tolerance??
            candidates = [p for p in (left, right, bottom, top) if p is not None]
            if not candidates:
                continue  # nothing to draw here
            dists = [np.linalg.norm(p - vertex) for p in candidates]
            far_point = candidates[np.argmax(dists)]

            # draw from the finite vertex out to the clipped boundary point
            line, = ax.plot([vor.vertices[i, 0], far_point[0]],
                        [vor.vertices[i, 1], far_point[1]],
                        'lightgray', lw=0.8, zorder=0)
                        
        voronoi_lines.append(line)


# Animation frame update function
def update(frame):
    sim.step()

    for i, bot in enumerate(swarm.robots):
        path = sim.robot_paths[i]
        xdata, ydata = zip(*path)
        scatters[i].set_data([xdata[-1]], [ydata[-1]])
        trails[i].set_data(xdata, ydata)

    draw_voronoi(seed_points, field)
    frame_text.set_text(f"Frame: {frame + 1}/{STEP_COUNT}")

    # final frame has no trails
    if frame == sim.steps - 1:
        for trail in trails:
            trail.set_data([], [])

    return scatters + trails + voronoi_lines + [frame_text]


# Run Animation
ani = FuncAnimation(fig, update, frames=STEP_COUNT, interval=ANIMATION_INTERVAL, blit=True, repeat=False)
plt.legend(loc = 'lower right')
plt.tight_layout()
#plt.show() # if we show the plot before we save the animation it will fuck up so put this after if u rly wanna see
ani.save("videos/voronoi_swarm_simulation.gif", fps=1000/ANIMATION_INTERVAL)
fig.savefig("voronoi_swarm_simulation.png", dpi=300, bbox_inches='tight')


