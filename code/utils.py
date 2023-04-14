def statistical_outlier_removal(pc, num_neighbors, std_ratio):
    # Statistical oulier removal
    cl, ind = pc.remove_statistical_outlier(nb_neighbors=num_neighbors,
                                                        std_ratio=std_ratio)
    inlier_cloud = pc.select_by_index(ind)
    
    return inlier_cloud, ind