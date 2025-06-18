from Environment.points_and_plane import Plane
from Environment.voronoi import VoronoiDiagram

plane = Plane(width=100, height=100, num_points=5)
plane.plot()

plane.add_point(25, 25)
plane.move_point(0, dx=10, dy=0)
plane.set_point_position(1, x=80, y=90)
plane.remove_point(2)
plane.plot()
points = plane.get_all_positions()

voronoi = VoronoiDiagram(points, width=plane.width, height=plane.height)
voronoi.compute()
voronoi.plot()

