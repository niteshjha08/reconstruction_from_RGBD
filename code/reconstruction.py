import open3d as o3d
import numpy as np

import pointcloud_to_mesh as pc2mesh
import utils
from reconstruction_preprocessor import ReconstructionPreprocessor
from reconstruction_postprocessor import ReconstructionPostprocessor

class Reconstruction:
    def __init__(self, scene_path, preprocessor_config, reconstruction_config, postprocessor_config, debug=True):
        self.scene_path = scene_path
        scene_name = scene_path.split('/')[-2]
        self.scene_name = scene_name
        self.pcd_path = scene_path + scene_name + '.pcd'
        self.point_labels_path = scene_path + scene_name + '.label'
        

        self.preprocessor = ReconstructionPreprocessor(preprocessor_config, debug)
        self.postprocessor = ReconstructionPostprocessor(postprocessor_config, debug)

        self.mesh_strategy = reconstruction_config['mesh_strategy']
        self.reconstruction_config = reconstruction_config
        self.debug = debug
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

    def estimate_normals(self, pcd):
        print("Estimate normals")
        pcd.estimate_normals()
        o3d.visualization.draw_geometries([pcd], point_show_normal=True)

        print("Orient normals consistently")
        pcd.orient_normals_consistent_tangent_plane(20)
        o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    def mesh_alpha_shape(self, pcd, alpha):
        print("Reconstruct a mesh with alpha shapes")
        print(f"alpha={alpha:.3f}")
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
        mesh.compute_vertex_normals()
        return mesh
        
    def mesh_ball_pivot(self, pcd):
        # print("Reconstruct a mesh with ball pivoting")
        # alpha = 0.05
        # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
        # mesh.compute_vertex_normals()
        # pcd = mesh.sample_points_poisson_disk(3000)
        
                
        radii = [0.005, 0.01, 0.02, 0.04]
        rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, o3d.utility.DoubleVector(radii))

        return rec_mesh
    
    def mesh_poisson(self, pcd):
        print("Reconstruct a mesh with Poisson surface reconstruction")
        with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
        mesh.compute_vertex_normals()

        return mesh, densities
    
    def run_reconstruction(self):
        # setup
        self.setup()

        # preprocessing
        print("Preprocessing")
        self.pcd = self.preprocessor.preprocess(self.pcd)
        print("Preprocessing complete")

        print("Reconstruction")
        densities = None
        # mesh generation
        if self.mesh_strategy == 'alpha_shape':
            self.mesh = self.mesh_alpha_shape(self.pcd, alpha=0.05)
        
        elif self.mesh_strategy == 'ball_pivot' or self.mesh_strategy=='poisson':
            if not self.pcd.has_normals():
                self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=0.1, max_nn=30))
                
                if self.reconstruction_config['normals_estimation_orientParam'] is not None:
                    self.pcd.orient_normals_consistent_tangent_plane(
                        self.reconstruction_config['normals_estimation_orientParam'])

            if self.mesh_strategy == 'ball_pivot':   
                self.mesh = self.mesh_ball_pivot(self.pcd)

            else:
                self.mesh, densities = self.mesh_poisson(self.pcd)
        
        else:
            print('Invalid mesh strategy')
            return
        print("Reconstruction complete")
        
        # Visualize mesh
        o3d.visualization.draw_geometries([self.mesh],mesh_show_back_face=True)
        
        # postprocessing
        print("Postprocessing")
        self.mesh = self.postprocessor.postprocess(self.pcd, self.mesh, self.reconstruction_config['mesh_strategy'], densities)
        print("Postprocessing complete")
        
        o3d.visualization.draw_geometries([self.mesh],mesh_show_back_face=True)