import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial import Voronoi, voronoi_plot_2d

from ross.core.environment import Field
from ross.core.swarm import Swarm
from ross.core.simulator import Simulator
from ross.behaviors.voronoi_balance import VoronoiBalanceBehavior

STEP_COUNT = 50 # Number of simulation steps
ANIMATION_INTERVAL = 200 # milliseconds per frame

# Initialize Environment, Swarm, Simulator
field = Field(width=100, height=100, num_points=7)
points = field.get_all_positions()

vor = Voronoi(points)

print("Voronoi vertices:", vor.vertices)
print("Voronoi ridge vertices:", vor.ridge_vertices)
print("Voronoi ridge points:", vor.ridge_points)

fig = voronoi_plot_2d(vor)
# ploit the ridge vertices
for ridge in vor.ridge_vertices:
        pt1, pt2 = vor.vertices[ridge]
        plt.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]])

# # plot the ridge points
for ridge in vor.ridge_points:
    pt1, pt2 = vor.points[ridge]
    plt.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]])
plt.show()