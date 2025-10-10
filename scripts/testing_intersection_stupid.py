import numpy as np
import matplotlib.pyplot as plt

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
    

# ---------------------------------------------------------------------
# Test scenarios: (description, line endpoints, ray start, ray direction)
tests = [
    ("Hits mid‑segment",
     np.array([0, 0]), np.array([4, 4]),
     np.array([2, -1]), np.array([0, 1])),
    
    ("Ray misses segment",
     np.array([0, 0]), np.array([4, 0]),
     np.array([2, -1]), np.array([1, -1])),
    
    ("Intersection beyond segment end",
     np.array([0, 0]), np.array([2, 0]),
     np.array([3, -1]), np.array([0, 1])),
    
    ("Parallel (no intersection)",
     np.array([0, 0]), np.array([4, 0]),
     np.array([0, 1]), np.array([4, 0])),
    
    ("Ray opposite direction (no intersection)",
     np.array([0, 0]), np.array([4, 4]),
     np.array([2, 6]), np.array([0, 1])),
]

for idx, (title, p1, p2, r_start, r_dir) in enumerate(tests, 1):
    pt = intersection(p1, p2, r_start, r_dir)

    # Set up a fresh figure for each test
    fig, ax = plt.subplots()
    ax.set_title(f"Test {idx}: {title}\nResult: {pt if pt is not None else 'None'}")

    # Plot line segment
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], marker='o', label="segment")

    # Draw ray as a long arrow for visualization
    ray_end = r_start + r_dir / np.linalg.norm(r_dir) * 10  # scale for display
    ax.plot([r_start[0], ray_end[0]], [r_start[1], ray_end[1]], marker='>', label="ray")
    #plot start point
    ax.plot(r_start[0], r_start[1], marker='o', color='red', label="ray start")

    # Mark the intersection if it exists
    if pt is not None:
        ax.plot(pt[0], pt[1], marker='x', markersize=8, label="intersection")

    ax.set_aspect("equal", adjustable='box')
    ax.grid(True)
    ax.legend()

plt.show()
