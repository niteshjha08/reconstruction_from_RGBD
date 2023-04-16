import open3d as o3d
import numpy as np
import os

from reconstruction_preprocessor import ReconstructionPreprocessor
from reconstruction_postprocessor import ReconstructionPostprocessor

""" 
Reconstruction class
"""
class Reconstruction:
    def __init__(self, scene_path, preprocessor_config, reconstruction_config, postprocessor_config, debug=True, save_mesh = True):
        self.scene_path = scene_path
        scene_name = scene_path.split('/')[-1]
        self.scene_name = scene_name

        self.pcd_path = os.path.join(scene_path, scene_name + '.pcd')
        # Can be used to reconstruct specific objects
        self.point_labels_path = os.path.join(scene_path, scene_name + '.label')

        self.preprocessor = ReconstructionPreprocessor(preprocessor_config, debug)
        self.postprocessor = ReconstructionPostprocessor(postprocessor_config, debug)

        self.mesh_strategy = reconstruction_config['mesh_strategy']
        self.reconstruction_config = reconstruction_config
        self.debug = debug
        self.save_mesh = save_mesh
        print("Reconstruction initialized")

    def get_labels(self):
        with open(self.point_labels_path, 'r') as f:
            point_labels = f.readlines()
        
        # strip the newline character and split the string with spaces, convert to int
        labels = [int(x.strip()) for x in point_labels[0].split(' ')]
        
        # remove the first number which is the number of points
        labels = np.array(labels[1:])
        return labels
    
    def setup(self):
        self.pcd = o3d.io.read_point_cloud(self.pcd_path)
        self.labels = self.get_labels()
        print("Setup Complete")

        """
        If the point cloud doesn't have normals, estimate them, and if the normals_estimation_orientParam is not None, orient them consistently
        """
    def estimate_normals(self):
        print("Estimate normals")
        self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.reconstruction_config['normals_estimation_radiusParam'], max_nn=self.reconstruction_config['normals_estimation_knnParam']))
                
        if self.reconstruction_config['normals_estimation_orientParam'] is not None:
            self.pcd.orient_normals_consistent_tangent_plane(
                self.reconstruction_config['normals_estimation_orientParam'])

    """
    Alpha Shapes mesh reconstruction
    """
    def mesh_alpha_shape(self, pcd, alpha):
        print("Reconstruct a mesh with alpha shapes")
        print(f"alpha={alpha:.3f}")
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
        mesh.compute_vertex_normals()
        return mesh
        
    """
    Ball Pivoting mesh reconstruction
    """
    def mesh_ball_pivot(self, pcd): 
        print("Reconstruct a mesh with ball pivoting")    
        rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, o3d.utility.DoubleVector(self.reconstruction_config['BPA_radii']))

        return rec_mesh
    
    """
    Poisson surface reconstruction
    """
    def mesh_poisson(self, pcd):
        print("Reconstruct a mesh with Poisson surface reconstruction")
        with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
        mesh.compute_vertex_normals()

        return mesh, densities
    
    """
    Main function to run the reconstruction
    """
    def run_reconstruction(self):
        # setup
        self.setup()

        # preprocessing to downsample and remove outliers
        print("Preprocessing")
        self.pcd = self.preprocessor.preprocess(self.pcd)
        print("Preprocessing complete")

        print("Reconstruction")
        densities = None
        # mesh generation
        if self.mesh_strategy == 'alpha_shape':
            self.mesh = self.mesh_alpha_shape(self.pcd, alpha=self.reconstruction_config['alpha_shape_alpha'])
        
        elif self.mesh_strategy == 'ball_pivot' or self.mesh_strategy=='poisson':
            if not self.pcd.has_normals():
                self.estimate_normals()

            if self.mesh_strategy == 'ball_pivot':   
                self.mesh = self.mesh_ball_pivot(self.pcd)

            else:
                self.mesh, densities = self.mesh_poisson(self.pcd)
        
        else:
            print('Invalid mesh strategy')
            return
        print("Reconstruction complete")
        
        # Visualize mesh
        if self.debug:
            o3d.visualization.draw_geometries([self.mesh],mesh_show_back_face=True)
        
        # postprocessing to smooth the mesh, crop to bbox of point cloud, and remove low density areas(only for poisson)
        print("Postprocessing")
        self.mesh = self.postprocessor.postprocess(self.pcd, self.mesh, self.reconstruction_config['mesh_strategy'], densities)
        print("Postprocessing complete")

        o3d.visualization.draw_geometries([self.mesh],mesh_show_back_face=True)

        if self.save_mesh:
            if not os.path.exists(self.reconstruction_config["output_path"]):
                os.makedirs(self.reconstruction_config["output_path"])
            o3d.io.write_triangle_mesh(self.reconstruction_config["output_path"] + self.scene_name + '.ply', self.mesh)