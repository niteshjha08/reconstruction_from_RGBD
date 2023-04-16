#!/usr/bin/env python3
from reconstruction import Reconstruction
import os
import time
import config
import apply_texture
import argparse

if __name__ == "__main__":

    argparse = argparse.ArgumentParser()
    argparse.add_argument('--aligned_dir_path', type=str, default=None, help='Path to the rgbd-scenes_aligned dir')
    argparse.add_argument('--images_path', type=str, default=None, help='Path to the RGBD images (only required if apply_texture is True)')
    argparse.add_argument('--scene_name', type=str, default=None, help='Path to the scene (ex: desk/desk_3)')
    argparse.add_argument('--debug', type=bool, default=False,help='Debug mode')

    argparse.add_argument('--save_mesh', type=bool, default=True,help='Save the mesh')

    args = argparse.parse_args()
    data_path = args.aligned_dir_path
    images_path = args.images_path
    scene_name = args.scene_name
    debug = args.debug
    save_mesh = args.save_mesh
    if images_path is not None:
        images_path = os.path.join(images_path, scene_name)

    scene_path = os.path.join(data_path, scene_name)
    print("data_path:", data_path)
    print("scene_path:", scene_path)
    print("images_path:", images_path)

    scene_name_split = scene_name.split('/')[-1]

    pose_file_path = os.path.join(scene_path, scene_name_split + '.pose')
    print("pose_file_path:", pose_file_path)

    # save mesh if applying texture
    save_mesh = save_mesh or config.apply_texture

    reconstruction = Reconstruction(scene_path, config.preprocessor_config, config.reconstruction_config, config.postprocessor_config, debug=debug, save_mesh=save_mesh)

    start_time = time.time()
    reconstruction.run_reconstruction()
    print(f"Reconstruction took {time.time() - start_time} seconds")

    if config.apply_texture and images_path is not None:
        scene_name = scene_path.split('/')[-1]
        if scene_name == '':
            scene_name = scene_path.split('/')[-2]

        mesh_path = os.path.join(config.reconstruction_config["output_path"], scene_name + '.ply')


        start_time = time.time()
        apply_texture.run(images_path,  mesh_path, pose_file_path, config.color_map_optimizer)
        print(f"Texture application took {time.time() - start_time} seconds")
