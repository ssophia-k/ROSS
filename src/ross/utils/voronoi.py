from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import numpy as np

#ok i dont use this shit rn

class VoronoiDiagram:
    def __init__(self, points: np.ndarray, width: int, height: int):
        self.points = points
        self.width = width
        self.height = height
        self.diagram = None

    def compute(self):
        if len(self.points) >= 3:  # Minimum required for Voronoi
            self.diagram = Voronoi(self.points)
        else:
            self.diagram = None
    
    def plot(self):
        if self.diagram is None:
            print("Not enough points to compute Voronoi diagram.")
            return

        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111)
        voronoi_plot_2d(self.diagram, ax=ax, show_vertices=False, line_colors='black', line_width=1.5, point_size=0)
        ax.scatter(self.points[:, 0], self.points[:, 1], c='blue', s=50, label='Seed Points')
        ax.set_xlim(0, self.width)
        ax.set_ylim(0, self.height)
        ax.set_title("Voronoi Diagram")
        ax.grid(True)
        ax.legend()
        plt.show()

    def get_finite_edges(self):
        """Returns a list of Voronoi edge segments within bounds."""
        edges = []
        if self.diagram is None:
            return edges

        for p1, p2 in self.diagram.ridge_vertices:
            if p1 == -1 or p2 == -1:
                continue  # Skip infinite ridges
            pt1 = self.diagram.vertices[p1]
            pt2 = self.diagram.vertices[p2]
            edges.append((pt1, pt2))
        return edges
    

