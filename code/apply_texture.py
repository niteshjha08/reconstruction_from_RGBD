#!/usr/bin/env python3
import os
import json
import open3d as o3d
import numpy as np
import re
from os.path import join
# Referred from http://www.open3d.org/docs/release/tutorial/pipelines/color_map_optimization.html

def sorted_alphanum(file_list_ordered):
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(file_list_ordered, key=alphanum_key)


def get_file_list(path, extension=None):
    if extension is None:
        file_list = [
            path + f for f in os.listdir(path) if os.path.isfile(join(path, f))
        ]
    else:
        file_list = [
            # path + f
            os.path.join(path, f)
            for f in os.listdir(path)
            if os.path.isfile(os.path.join(path, f)) and
            os.path.splitext(f)[1] == extension
        ]
    file_list = sorted_alphanum(file_list)

    return file_list

def data_loader(path):
    rgbd_images = []
    all_images = get_file_list(path, extension=".png")
    depth_image_path = []
    color_image_path = []
    for image in all_images:
        if "depth" in image:
            depth_image_path.append(image)
        else:
            color_image_path.append(image)

    assert (len(depth_image_path) == len(color_image_path))
    for i in range(len(depth_image_path)):
        depth = o3d.io.read_image(os.path.join(depth_image_path[i]))
        color = o3d.io.read_image(os.path.join(color_image_path[i]))
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, convert_rgb_to_intensity=False)

        rgbd_images.append(rgbd_image)
    return rgbd_images

is_ci = False


def quaternion_to_rotation_matrix(q):
    """Convert quaternion to rotation matrix.
    Args:
        q: (4,) array, quaternion.
    Returns:
        R: (3, 3) array, rotation matrix.
    """
    q = np.array(q)
    assert q.shape == (4,), "q must be (4,)"
    q /= np.linalg.norm(q)
    w, x, y, z = q
    R = np.array([[1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * z * w,
                   2 * x * z + 2 * y * w],
                  [2 * x * y + 2 * z * w, 1 - 2 * x**2 - 2 * z**2,
                   2 * y * z - 2 * x * w],
                  [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w,
                   1 - 2 * x**2 - 2 * y**2]])
    return R

def generate_camera_trajectory(pose_file_path):
    f = open(pose_file_path, "r")
    traj = open("traj.log", "w")
    lines = f.readlines()
    for line in lines:
        line = line.split(" ")
        line = [float(i) for i in line[2:]]
        rot = quaternion_to_rotation_matrix(line[:4])
        rot = np.vstack((rot, np.array([0, 0, 0])))
        rot = np.hstack((rot, np.array([[line[4]], [line[5]], [line[6]], [1]])))

        traj.write(f"0 0 0\n"
                   f"{rot[0, 0]} {rot[0, 1]} {rot[0, 2]} {rot[0, 3]}\n"
                   f"{rot[1, 0]} {rot[1, 1]} {rot[1, 2]} {rot[1, 3]}\n"
                   f"{rot[2, 0]} {rot[2, 1]} {rot[2, 2]} {rot[2, 3]}\n"
                   f"{rot[3, 0]} {rot[3, 1]} {rot[3, 2]} {rot[3, 3]}\n"
                   )


def run(images_path, mesh_path, pose_file_path, color_map_optimizer):

    mesh_save_name = mesh_path.split('.ply')[0]

    rgbd_images = data_loader(images_path)

    generate_camera_trajectory(pose_file_path)
    camera_trajectory_path = "traj.log"

    camera_trajectory = o3d.io.read_pinhole_camera_trajectory(camera_trajectory_path)
    mesh = o3d.io.read_triangle_mesh(mesh_path)

    if color_map_optimizer == "rigid":
        print("rigid optimization")
        maximum_iteration = 100 if is_ci else 300
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            mesh, camera_trajectory = o3d.pipelines.color_map.run_rigid_optimizer(
                mesh, rgbd_images, camera_trajectory,
                o3d.pipelines.color_map.RigidOptimizerOption(
                    maximum_iteration=maximum_iteration))

        # Save mesh
        mesh_save_path = mesh_save_name + '_rigid.ply'
        o3d.io.write_triangle_mesh(mesh_save_path, mesh)
    elif color_map_optimizer == "non_rigid":
        print("non-rigid optimization")
        maximum_iteration = 100 if is_ci else 300
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            mesh, camera_trajectory = o3d.pipelines.color_map.run_non_rigid_optimizer(
                mesh, rgbd_images, camera_trajectory,
                o3d.pipelines.color_map.NonRigidOptimizerOption(
                    maximum_iteration=maximum_iteration))
            
        # Save mesh
        mesh_save_path = mesh_save_name + '_non_rigid.ply'
        o3d.io.write_triangle_mesh(mesh_save_path, mesh)
        
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
