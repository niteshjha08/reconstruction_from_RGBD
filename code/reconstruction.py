import open3d as o3d
import cv2
import numpy as np
import os

import pointcloud_to_mesh as pc2mesh
import utils

class Reconstruction:
    def __init__(self, scene_path, statistical_outlier_removal=True, alpha_shape=True, ball_pivot=False):
        self.scene_path = scene_path
        scene_name = scene_path.split('/')[-2]
        self.scene_name = scene_name
        self.pcd_path = scene_path + scene_name + '.pcd'
        self.pcd = o3d.io.read_point_cloud(self.pcd_path)
        self.point_labels_path = scene_path + scene_name + '.label'
        self.labels = self.get_labels()
        self.statistical_outlier_removal = statistical_outlier_removal
        self.alpha_shape = alpha_shape
        self.ball_pivot = ball_pivot


    def get_labels(self):
        with open(self.point_labels_path, 'r') as f:
            point_labels = f.readlines()
        
        # strip the newline character and split the string with spaces, convert to int
        labels = [int(x.strip()) for x in point_labels[0].split(' ')]
        
        # remove the first number which is the number of points
        labels = np.array(labels[1:])
        return labels
    
    def run_reconstruction(self):
        # Statistical outlier removal
        if self.statistical_outlier_removal:
            self.pcd, ind = utils.statistical_outlier_removal(self.pcd, num_neighbors=20, std_ratio=2.0)
            # display_inlier_outlier(self.pcd, ind)
        
        # Alpha shape
        if self.alpha_shape:
            self.mesh = pc2mesh.mesh_alpha_shape(self.pcd, alpha=0.05)
        
        # Ball pivot
        elif self.ball_pivot:
            self.mesh = pc2mesh.mesh_ball_pivot(self.pcd)
        
        
        # # Save mesh
        # mesh_path = self.scene_path + self.scene_name + '.ply'
        # o3d.io.write_triangle_mesh(mesh_path, self.mesh)
        
        # Visualize mesh
        o3d.visualization.draw_geometries([self.mesh],mesh_show_back_face=True)
        
        # # Save mesh as png
        # mesh_png_path = self.scene_path + self.scene_name + '.png'
        # mesh.compute_vertex_normals()
        # mesh.compute_triangle_normals()
        # o3d.visualization.draw_geometries_with_editing([mesh])
        # o3
