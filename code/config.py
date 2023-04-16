# Configuration parameters

preprocessor_config = dict()
preprocessor_config['downsample_voxel'] = 0.01
# Statistical outlier removal parameters, set to None to disable
preprocessor_config['outlier_num_neighbor'] = 20
preprocessor_config['outlier_std_ratio'] = 2.0


reconstruction_config = dict()
# Possible values: 'alpha_shape', 'poisson', 'ball_pivoting'
reconstruction_config['mesh_strategy'] = 'poisson'
reconstruction_config['normals_estimation_knnParam'] = 30
# Set to None to disable
reconstruction_config['normals_estimation_orientParam'] = 20


postprocessor_config = dict()
# Possible values: 'simple', 'laplacian', 'taubin'
postprocessor_config['smooth_strategy'] = 'taubin'
postprocessor_config['smooth_iterations'] = 10
postprocessor_config['density_threshold'] = 0.1
