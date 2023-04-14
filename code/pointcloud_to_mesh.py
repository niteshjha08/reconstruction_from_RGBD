import open3d as o3d

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