#!/usr/bin/env python3
import open3d as o3d

def statistical_outlier_removal(pc, num_neighbors, std_ratio):
    # Statistical oulier removal
    cl, ind = pc.remove_statistical_outlier(nb_neighbors=num_neighbors,
                                                        std_ratio=std_ratio)
    inlier_cloud = pc.select_by_index(ind)
    
    return inlier_cloud, ind

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

def smooth_mesh(mesh, num_iterations=5):
    
    # Smooth the mesh
    for i in range(num_iterations):
        mesh = mesh.filter_smooth_taubin(num_iterations)
    return mesh

def mesh_smoothing():
    mesh = o3d.io.read_triangle_mesh("/home/nitesh/programming/take_home/reconstruction/possion_mesh.ply")
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

    mesh = smooth_mesh(mesh)
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

mesh_smoothing()