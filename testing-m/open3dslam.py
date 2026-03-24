import open3d as o3d

def main():
    print("Generating 3D sphere...")
    
    # 1. Create a sphere with a radius of 1.0
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    
    # 2. Compute normals so it reacts to lighting (otherwise it looks like a flat 2D circle)
    sphere.compute_vertex_normals()
    
    # 3. Paint it a nice color (RGB format, 0.0 to 1.0) - let's make it blue
    sphere.paint_uniform_color([0.1, 0.5, 0.8])
    
    print("Opening window... Click and drag to rotate, scroll to zoom.")
    print("Close the window to end the script.")
    
    # 4. Draw it! This blocks the script until you close the window.
    o3d.visualization.draw_geometries(
        [sphere], 
        window_name="Mac Open3D Test", 
        width=800, 
        height=600
    )

if __name__ == "__main__":
    main()