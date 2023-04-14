#!/usr/bin/env python3
from reconstruction import Reconstruction
import os
import time

if __name__ == "__main__":
    data_path = '/home/nitesh/programming/pediametrix/reconstruction/data/rgbd-scenes_aligned'

    scene_path = data_path + '/desk/desk_3/'
    
    reconstruction = Reconstruction(scene_path)
    start_time = time.time()
    reconstruction.run_reconstruction()
    print(f"Reconstruction took {time.time() - start_time} seconds")