#!/usr/bin/env python3
import open3d as o3d
import os
import numpy as np

def visualize_scene_pc(scene_path):
    scene_name = scene_path.split('/')[-2]
    pcd_path = scene_path + scene_name + '.pcd'

    pcd = o3d.io.read_point_cloud(pcd_path)

    # visualize pointcloud
    o3d.visualization.draw_geometries([pcd])

def visualize_object_pc(scene_path, object_index):
    scene_name = scene_path.split('/')[-2]
    pcd_path = scene_path + scene_name + '.pcd'

    pcd = o3d.io.read_point_cloud(pcd_path)
    point_labels_path = scene_path + scene_name + '.label'

    with open(point_labels_path, 'r') as f:
        point_labels = f.readlines()
    
    # strip the newline character and split the string with spaces, convert to int
    labels = [int(x.strip()) for x in point_labels[0].split(' ')]

    # remove the first number which is the number of points
    labels = np.array(labels[1:])

    object_points = np.where(labels == object_index)[0]
    
    if len(object_points) == 0:
        print('No object found')
        return

    object_pc = o3d.geometry.PointCloud.select_by_index(pcd, object_points)

    o3d.visualization.draw_geometries([object_pc])

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    
def visualize_scene(scene_path):

    scene_name = scene_path.split('/')[-2]
    pcd_path = scene_path + scene_name + '.pcd'
    print(pcd_path)
    pcd = o3d.io.read_point_cloud(pcd_path)
    print(pcd)
    print(np.asarray(pcd.points))
    # visualize pointcloud
    o3d.visualization.draw_geometries([pcd])

    # visualize mesh
    alpha = 0.05
    print(f"alpha={alpha:.3f}")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

# 0 is background, 1 is bowl, 2 is cap, 3 is cereal box, 4 is coffee mug, and 5 is soda can
def visualize_object(scene_path, object_index):
    
        scene_name = scene_path.split('/')[-2]
        pcd_path = scene_path + scene_name + '.pcd'

        pcd = o3d.io.read_point_cloud(pcd_path)
        point_labels_path = scene_path + scene_name + '.label'

        with open(point_labels_path, 'r') as f:
            point_labels = f.readlines()
        
        # strip the newline character and split the string with spaces, convert to int
        labels = [int(x.strip()) for x in point_labels[0].split(' ')]

        # remove the first number which is the number of points
        labels = np.array(labels[1:])

        object_points = np.where(labels == object_index)[0]
        
        if len(object_points) == 0:
            print('No object found')
            return

        object_pc = o3d.geometry.PointCloud.select_by_index(pcd, object_points)

        o3d.visualization.draw_geometries([object_pc])
        print("Statistical oulier removal")
        cl, ind = object_pc.remove_statistical_outlier(nb_neighbors=50,
                                                            std_ratio=1.0)
        # display_inlier_outlier(object_pc, ind)
        inlier_cloud = object_pc.select_by_index(ind)
        o3d.visualization.draw_geometries([inlier_cloud])
        # mesh_alpha_shape(inlier_cloud)
        mesh_ball_pivot(inlier_cloud)




if __name__ == '__main__':

    data_path = '/home/nitesh/programming/pediametrix/reconstruction/data/rgbd-scenes_aligned'

    scene_path = data_path + '/desk/desk_3/'
    # scene_path = data_path + '/kitchen_small/kitchen_small_1/'
    # scene_path = data_path + '/meeting_small/meeting_small_1/'
    # scene_path = data_path + '/table/table_1/'
    # scene_path = data_path + '/table_small/table_small_2/'


    visualize_scene(scene_path)

    # visualize_object(scene_path, 1)