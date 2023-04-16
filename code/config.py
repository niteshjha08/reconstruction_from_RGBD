# Configuration parameters

preprocessor_config = dict()
preprocessor_config['downsample_voxel'] = 0.01
# Statistical outlier removal parameters, set to None to disable
preprocessor_config['outlier_num_neighbor'] = 20
preprocessor_config['outlier_std_ratio'] = 2.0


reconstruction_config = dict()
# Possible values: 'alpha_shape', 'poisson', 'ball_pivot'
reconstruction_config['mesh_strategy'] = 'poisson'
reconstruction_config['normals_estimation_knnParam'] = 30
reconstruction_config['normals_estimation_radiusParam'] = 0.1
# Set to None to disable
reconstruction_config['normals_estimation_orientParam'] = 20
# Mesh parameters
reconstruction_config['poisson_depth'] = 9
reconstruction_config['alpha_shape_alpha'] = 0.05
reconstruction_config['BPA_radii'] = [0.05, 0.04, 0.02, 0.03]
# Ouput path
reconstruction_config["output_path"] = './outputs/'

postprocessor_config = dict()
# Possible values: 'simple', 'laplacian', 'taubin'
postprocessor_config['smooth_strategy'] = 'taubin'
postprocessor_config['smooth_iterations'] = 10
postprocessor_config['density_threshold'] = 0.1

apply_texture = True
# Possible values: 'rigid', 'non_rigid'
color_map_optimizer = 'rigid'