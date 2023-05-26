
### Requirements:
1. RGBD Aligned Dataset: download from here https://rgbd-dataset.cs.washington.edu/dataset/rgbd-scenes_aligned/
2. If Texture is required, download the RGBD dataset from here: https://rgbd-dataset.cs.washington.edu/dataset/rgbd-scenes/

### Configuration of reconstruction:
code/config.py contains all configuration parameters that are used in the project. This consists of configs for preprocessing , reconstruction, postprocessing, and applying texture.  To use values other than the current ones, modify them manually in code/config.py

### Steps to run
1. cd to project root 'reconstruction'
2. Execute `python3 code/main.py --aligned_dir_path <path_to_rgbd-scenes_aligned dir> --images_path <path_to_rgbd-scenes_dir> --scene_name <scene_name> --debug <bool> --save_mesh <bool>`

If texture is not required: `python3 code/main.py --aligned_dir_path <path_to_rgbd-scenes_aligned dir> --scene_name <scene_name> --debug <bool> --save_mesh <bool>`

scene_name can be of the following format from the 8 scenes: 
- desk/desk_1
- kitchen_small/kitchen_small_1
- meeting_small/meeting_small_1
- etc.

Example: python3 code/main.py --aligned_dir_path /home/sample_path/reconstruction/data/rgbd-scenes_aligned --images_path /home/sample_path2/rgbd-scenes --scene_name desk/desk_3 --debug True --save_mesh True

### Pipeline
1. Input pointcloud

![Input_pc](https://github.com/niteshjha08/reconstruction_from_RGBD/blob/main/media/1_input.png)

2. Voxel downsampling 

![voxel_downsample](https://github.com/niteshjha08/reconstruction_from_RGBD/blob/main/media/2_downsample.png)

3. Statistical Outlier Removal 
![outlier](https://github.com/niteshjha08/reconstruction_from_RGBD/blob/main/media/3_outlier.png)

![outlier_removal](https://github.com/niteshjha08/reconstruction_from_RGBD/blob/main/media/4_remove_outlier.png)

4. Mesh Generation (using Poisson's surface reconstruction) 

![poissons](https://github.com/niteshjha08/reconstruction_from_RGBD/blob/main/media/5_poisson.png)

5. Low density regions identification

![low_density](https://github.com/niteshjha08/reconstruction_from_RGBD/blob/main/media/6_density.png)

6. Low density region removal

![low_density_removal](https://github.com/niteshjha08/reconstruction_from_RGBD/blob/main/media/7_density_thresh.png)

7. Cropping with PointCloud Bounding Box

![crop](https://github.com/niteshjha08/reconstruction_from_RGBD/blob/main/media/8_crop.png)

8. Mesh Smoothing

![Mesh Smoothing](https://github.com/niteshjha08/reconstruction_from_RGBD/blob/main/media/9_smooth.png)

### Final Result

![final](https://github.com/niteshjha08/reconstruction_from_RGBD/blob/main/media/final.gif)