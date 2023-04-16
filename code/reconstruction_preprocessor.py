import open3d as o3d
import utils

class ReconstructionPreprocessor:
    def __init__(self, preprocessor_config, debug = False):
        self.downsample_voxel_size = preprocessor_config['downsample_voxel']
        self.num_neighbors = preprocessor_config['outlier_num_neighbor']
        self.std_ratio = preprocessor_config['outlier_std_ratio']
        self.debug = debug

    def statistical_outlier_removal(self, pc):
        cl, ind = pc.remove_statistical_outlier(nb_neighbors=self.num_neighbors,
                                                            std_ratio=self.std_ratio)
        inlier_cloud = pc.select_by_index(ind)
        
        return inlier_cloud, ind

    def preprocess(self, pcd):
        # Voxel downsample
        print("Downsample with a voxel size %.3f" % self.downsample_voxel_size)
        if self.downsample_voxel_size:
            pcd = pcd.voxel_down_sample(voxel_size=self.downsample_voxel_size)
        print("statistical outlier removal")
        # Statistical outlier removal
        if self.num_neighbors or self.std_ratio:
            pcd, ind = self.statistical_outlier_removal(pcd)
            # if self.debug:
            #     utils.display_inlier_outlier(pcd, ind)

        return pcd

    