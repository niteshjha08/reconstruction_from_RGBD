#!/usr/bin/env python3
from reconstruction import Reconstruction
import os
import time
import config

if __name__ == "__main__":
    data_path = '/home/nitesh/programming/take_home/reconstruction/data/rgbd-scenes_aligned'

    scene_path = data_path + '/desk/desk_3/'
    
    reconstruction = Reconstruction(scene_path, config.preprocessor_config, config.reconstruction_config, config.postprocessor_config, debug=True)
    start_time = time.time()
    reconstruction.run_reconstruction()
    print(f"Reconstruction took {time.time() - start_time} seconds")

    #TODO:
    """
    1. Smooth the meshes -- DONE
    2. UV project - try Blender? -- DONE
    3. Pros and cons of each method -- TODO - 1 hour
    4. Try different scenes -- TODO - 1 hour
    5. Metrics for accuracy of reconstruction --TODO - 1 hour
    6. Command line arguments -- TODO
    7. Code Structure finalize and clean up -- IN PROGRESS ~ 1.5 hours
    """