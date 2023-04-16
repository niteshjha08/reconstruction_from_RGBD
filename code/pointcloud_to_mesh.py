import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def mesh_alpha_shape(pcd, alpha):
    print("Reconstruct a mesh with alpha shapes")
    print(f"alpha={alpha:.3f}")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    # o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    return mesh

def mesh_ball_pivot(pcd):
    print("Reconstruct a mesh with ball pivoting")
    # radii = [0.005, 0.01, 0.02, 0.04]
    # for radius in radii:
    #     print(f"radius={radius:.3f}")
    #     mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    #         pcd, o3d.utility.DoubleVector([radius, radius * 2]))
    #     mesh.compute_vertex_normals()
    #     o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    alpha = 0.05
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    pcd = mesh.sample_points_poisson_disk(3000)

    radii = [0.005, 0.01, 0.02, 0.04]
    rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii))
    # o3d.visualization.draw_geometries([pcd, rec_mesh])
    return rec_mesh

def mesh_poisson(pcd):
    bbox = pcd.get_axis_aligned_bounding_box()
       
    
    print("Estimate normals")
    pcd.estimate_normals()
    o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    print("Orient normals consistently")
    pcd.orient_normals_consistent_tangent_plane(20)
    o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    print("Reconstruct a mesh with Poisson surface reconstruction")
    with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
    mesh.compute_vertex_normals()

    densities = np.asarray(densities)
    density_colors = plt.get_cmap('plasma')(
        (densities - densities.min()) / (densities.max() - densities.min()))
    density_colors = density_colors[:, :3]
    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)

    # p_mesh_crop = mesh.crop(bbox)
    # density_mesh_crop = density_mesh.crop(bbox)
    # o3d.visualization.draw_geometries([p_mesh_crop, density_mesh_crop], mesh_show_back_face=True)
    # o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    # return p_mesh_crop, density_mesh_crop
    return mesh, density_mesh, densities

# print('run Poisson surface reconstruction')
# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         pcd, depth=9)