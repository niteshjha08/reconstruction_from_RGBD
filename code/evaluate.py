import open3d as o3d
import numpy as np

def get_mesh_pc_distances(mesh, pc, visualize=False):
    """Get the distances between the mesh and the point cloud.
    Args:
        mesh (open3d.geometry.TriangleMesh): The mesh.
        pc (open3d.geometry.PointCloud): The point cloud.
    Returns:
        distances (np.array): The distances between the mesh and the point cloud.
    """
    # Convert the mesh to a point cloud
    mesh_pc = o3d.geometry.PointCloud()
    mesh_pc.points = mesh.vertices

    # Get the distances between the mesh and the point cloud
    distances = mesh_pc.compute_point_cloud_distance(pc)
    distances = np.asarray(distances)

    if visualize:
        mesh_pc.paint_uniform_color([0,0,1])
        pc.paint_uniform_color([0.5,0.5,0])   
        o3d.visualization.draw_geometries([mesh_pc, pc])

    return distances

def visualize_textures():
    mesh = o3d.io.read_triangle_mesh("possion_mesh.ply")
    mesh_r = o3d.io.read_triangle_mesh("possion_mesh_rigid.ply")
    mesh_nr = o3d.io.read_triangle_mesh("possion_mesh_nonrigid.ply")
    o3d.visualization.draw_geometries([mesh], window_name="Original")
    o3d.visualization.draw_geometries([mesh_r], window_name="Rigid")
    o3d.visualization.draw_geometries([mesh_nr], window_name="Non-Rigid")

    # visualize three meshes together
    # vis1 = o3d.visualization.Visualizer()
    # vis1.create_window(window_name="Original")
    # vis1.add_geometry(mesh)
    # vis2 = o3d.visualization.Visualizer()
    # vis2.create_window(window_name="Rigid")
    # vis2.add_geometry(mesh_r)
    # vis3 = o3d.visualization.Visualizer()
    # vis3.create_window(window_name="Non-Rigid")
    # vis3.add_geometry(mesh_nr)

    # while True:
    #     vis1.update_geometry()
    #     if not vis1.poll_events():
    #         break
    #     vis1.update_renderer()

    #     vis2.update_geometry()
    #     if not vis2.poll_events():
    #         break
    #     vis2.update_renderer()

    #     vis3.update_geometry()
    #     if not vis3.poll_events():
    #         break
    #     vis3.update_renderer()

    

visualize_textures()

