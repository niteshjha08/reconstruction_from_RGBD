import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

class ReconstructionPostprocessor:
    def __init__(self, postprocessor_config, debug = False):
        self.debug = debug
        self.postprocessor_config = postprocessor_config

    def low_density_removal(self, mesh, densities):
        densities = np.asarray(densities)
        density_colors = plt.get_cmap('plasma')(
            (densities - densities.min()) / (densities.max() - densities.min()))
        density_colors = density_colors[:, :3]
        density_mesh = o3d.geometry.TriangleMesh()
        density_mesh.vertices = mesh.vertices
        density_mesh.triangles = mesh.triangles
        density_mesh.triangle_normals = mesh.triangle_normals
        density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)

        # crop low density regions
        vertices_to_remove = densities < np.quantile(densities, self.postprocessor_config['density_threshold'])
        mesh.remove_vertices_by_mask(vertices_to_remove)
        return mesh

    def postprocess(self, pcd, mesh, mesh_strategy, densities = None):
        # Low density removal
        if mesh_strategy == 'poisson':
            mesh = self.low_density_removal(mesh, densities)

        # Crop mesh
        print("Crop mesh")
        mesh = mesh.crop(pcd.get_axis_aligned_bounding_box())
        
        # Mesh smoothing
        print("Smooth mesh")

        if self.postprocessor_config['smooth_strategy'] == 'laplacian':
            mesh = mesh.filter_smooth_laplacian(number_of_iterations=self.postprocessor_config['smooth_iterations'])

        elif self.postprocessor_config['smooth_strategy'] == 'taubin':
            mesh = mesh.filter_smooth_taubin(number_of_iterations=self.postprocessor_config['smooth_iterations'])

        elif self.postprocessor_config['smooth_strategy'] == 'simple':
            mesh = mesh.filter_smooth_simple(number_of_iterations=self.postprocessor_config['smooth_iterations'])

        return mesh
        