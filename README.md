
### Requirements:
1. RGBD Aligned Dataset: download from here https://rgbd-dataset.cs.washington.edu/dataset/rgbd-scenes_aligned/
2. If Texture is required, download the RGBD dataset from here: https://rgbd-dataset.cs.washington.edu/dataset/rgbd-scenes/

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