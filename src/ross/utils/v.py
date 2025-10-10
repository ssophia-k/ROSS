import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi

from ross.core.environment import Field

# Voronoi line storage
voronoi_lines = []


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


